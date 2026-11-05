/*
  SENSORES
  Inicialización y lectura de IMU, magnetómetro, barómetro y GPS.
  Interfaz para el programa principal:
    iniciarSensores(), procesarGPS(), leerSensores(), obtenerAlturaBarometrica()

  Los archivos .ino de una carpeta Arduino se compilan juntos; no se necesitan
  headers/prototipos adicionales. No compilar estas pestañas por separado.
*/

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <LPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>

#define INTERVALO_COMPROBACION_MS 200 // Diagnóstico I2C cada 200 ms
#define TIMEOUT_GPS_MS 2000
#define TIMEOUT_BAR_CONGELADO_MS 2500 // Solo se aplica a estados de vuelo

//////////////////////////////////// Configuración IMU LSM6DSO / LSM6DSOX (ID 0x6C)
  #define IMU_DIRECCION_I2C       0x6A
  #define IMU_RANGO_ACEL_G        16      // 2, 4, 8, 16 g (recomendado 16 para el despegue)
  #define IMU_RANGO_GIRO_DPS      2000    // 125, 250, 500, 1000, 2000 °/s
  #define IMU_FRECUENCIA_ACEL_HZ  416     // 13, 26, 52, 104, 208, 416, 833, 1660
  #define IMU_FRECUENCIA_GIRO_HZ  416     // mismas frecuencias
  #define IMU_FILTRO_ACEL_DIV_ODR 0       // 0=sin LPF2; 4,10,20,45,100,200,400,800
  #define IMU_FILTRO_GIRO_ACTIVO  0       // 0=desactivado; 1=activar LPF1
  #define IMU_FILTRO_GIRO_CODIGO  0       // 0..7 (FTYPE; frecuencia depende de ODR)
  #define IMU_REINTENTO_MS        2000    // Reintento de inicialización solo en tierra

  static_assert(IMU_RANGO_ACEL_G == 2 || IMU_RANGO_ACEL_G == 4 ||
                IMU_RANGO_ACEL_G == 8 || IMU_RANGO_ACEL_G == 16, "Rango IMU acc invalido");
  static_assert(IMU_RANGO_GIRO_DPS == 125 || IMU_RANGO_GIRO_DPS == 250 ||
                IMU_RANGO_GIRO_DPS == 500 || IMU_RANGO_GIRO_DPS == 1000 ||
                IMU_RANGO_GIRO_DPS == 2000, "Rango IMU gyro invalido");
  static_assert(IMU_FILTRO_GIRO_CODIGO >= 0 && IMU_FILTRO_GIRO_CODIGO <= 7,
                "Filtro gyro fuera de rango");
  static_assert(IMU_FILTRO_GIRO_ACTIVO == 0 || IMU_FILTRO_GIRO_ACTIVO == 1,
                "El filtro gyro se activa con 0 o 1");
  static_assert(IMU_FRECUENCIA_ACEL_HZ == 13 || IMU_FRECUENCIA_ACEL_HZ == 26 ||
                IMU_FRECUENCIA_ACEL_HZ == 52 || IMU_FRECUENCIA_ACEL_HZ == 104 ||
                IMU_FRECUENCIA_ACEL_HZ == 208 || IMU_FRECUENCIA_ACEL_HZ == 416 ||
                IMU_FRECUENCIA_ACEL_HZ == 833 || IMU_FRECUENCIA_ACEL_HZ == 1660,
                "ODR acelerometro no disponible");
  static_assert(IMU_FRECUENCIA_GIRO_HZ == 13 || IMU_FRECUENCIA_GIRO_HZ == 26 ||
                IMU_FRECUENCIA_GIRO_HZ == 52 || IMU_FRECUENCIA_GIRO_HZ == 104 ||
                IMU_FRECUENCIA_GIRO_HZ == 208 || IMU_FRECUENCIA_GIRO_HZ == 416 ||
                IMU_FRECUENCIA_GIRO_HZ == 833 || IMU_FRECUENCIA_GIRO_HZ == 1660,
                "ODR giroscopio no disponible");
////////////////////////////////////

#define GPS_ON  10
#define GPS_RXD 30
#define GPS_TXD 31
#define UART_GPS Serial1

// Ejes de placa e IMU: X hacia arriba, Y izquierda, Z fuera de placa.
// Estructura de muestras original, compartida con Telemetria.ino.
  union unionDatos{
    struct dato{   
      float acc[3], gyro[3], magn[3];
      float latitud, longitud, altitud, velocidad;
      char tiempo[3];//Tiene segundo, minuto, hora
      char fecha[3]; //Tiene día, mes, año-2000
      float hdop; //hdop y buscar
      uint16_t tiempoDesdeUltimoFix;
      bool gpsValido;
      float presion, altitud_barometro, temperatura;
      char estado; //estado
    } dato;
    char byte[sizeof(dato)];
  } dSt;


TinyGPSPlus gps;
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
sensors_event_t event;
LPS ps;
char recepcionGPS[512];

// Estado independiente de cada sensor. No se reinicializa automáticamente.
bool imuInicializada = false, magInicializado = false, barInicializado = false;
bool imuConectada = false, magConectado = false, barConectado = false;
bool gpsComunicando = false;
uint32_t ultimaTramaGPS = 0;
uint8_t idIMU = 0, ctrlAcelIMU = 0, ctrlGiroIMU = 0;
bool ultimaLecturaIMUValida = false;
bool barLecturaFresca = false;
uint32_t ultimaVariacionPresionMs = 0;
uint32_t muestrasIMUSinActualizar = 0;
uint32_t erroresLecturaIMU = 0;

// En Telemetria.ino se traduce este estado a los 8 bits del paquete LoRa.
void actualizarFlagsSensores(bool gpsReciente, bool gpsPreciso, bool imuOK, bool magOK, bool barOK);

// Offsets originales, en las mismas unidades de cada librería.
float offsetAccel[] = {-0.004, 0.0065, -0.0145};
float offsetGiro[]  = {-0.52, 0.64, 0.03};
float offsetMagn[]  = {-78.0, -92.0, +15.5};

// Utilizados solo si se activa la opción de depuración de medias.
float gananciaParaLaMedia = 0.02f;
float mediaAcc[3], mediaGiro[3], mediaMagn[3];

//////////////////////////////////// Comunicación I2C directa IMU (igual que el test probado)
  bool leerIdentificadorI2C(uint8_t direccion, uint8_t registro, uint8_t &id);

  bool escribirRegistroIMU(uint8_t registro, uint8_t valor) {
    Wire.beginTransmission(IMU_DIRECCION_I2C);
    Wire.write(registro);
    Wire.write(valor);
    return Wire.endTransmission() == 0;
  }

  uint8_t codigoODRIMU(int hz) {
    switch (hz) {
      case 13: return 1; case 26: return 2; case 52: return 3;
      case 104: return 4; case 208: return 5; case 416: return 6;
      case 833: return 7; case 1660: return 8; default: return 0;
    }
  }

  uint8_t codigoFiltroAcel() {
    switch (IMU_FILTRO_ACEL_DIV_ODR) {
      case 0: return 0xFF; // Desactivado
      case 4: return 0; case 10: return 1; case 20: return 2;
      case 45: return 3; case 100: return 4; case 200: return 5;
      case 400: return 6; case 800: return 7; default: return 0xFE;
    }
  }

  float escalaAcelIMU() { return 0.0000305f * IMU_RANGO_ACEL_G; } // g/LSB

  float escalaGiroIMU() { // grados/s por LSB
    switch (IMU_RANGO_GIRO_DPS) {
      case 125: return 0.004375f;
      case 250: return 0.00875f;
      case 500: return 0.0175f;
      case 1000: return 0.035f;
      default: return 0.070f;
    }
  }

  bool iniciarIMU() {
    imuInicializada = imuConectada = ultimaLecturaIMUValida = false;
    uint8_t ctrl8 = 0, ctrl4 = 0, ctrl6 = 0;
    if (!leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x0F, idIMU) || idIMU != 0x6C) return false;

    const uint8_t odrAcc = codigoODRIMU(IMU_FRECUENCIA_ACEL_HZ);
    const uint8_t odrGir = codigoODRIMU(IMU_FRECUENCIA_GIRO_HZ);
    const uint8_t filtroAcc = codigoFiltroAcel();
    if (!odrAcc || !odrGir || filtroAcc == 0xFE) return false;

    // LSM6DSO: ±2=0, ±16=1, ±4=2, ±8=3.
    const uint8_t rangoAcc = IMU_RANGO_ACEL_G == 2 ? 0 :
                             IMU_RANGO_ACEL_G == 4 ? 2 :
                             IMU_RANGO_ACEL_G == 8 ? 3 : 1;
    // CTRL2_G: FS_G[1:0] en bits 3:2; FS_125 en bit 1.
    const uint8_t rangoGir = IMU_RANGO_GIRO_DPS == 125 ? 0x02 :
                             IMU_RANGO_GIRO_DPS == 500 ? 0x04 :
                             IMU_RANGO_GIRO_DPS == 1000 ? 0x08 :
                             IMU_RANGO_GIRO_DPS == 2000 ? 0x0C : 0x00;
    if (!leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x17, ctrl8) ||
        !leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x13, ctrl4) ||
        !leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x15, ctrl6)) return false;

    // CTRL3_C: BDU=1, autoincremento=1. CTRL8_XL: XL_FS_MODE=0, LPF2 opcional.
    const uint8_t nuevoCtrl8 = (ctrl8 & 0x19) | (filtroAcc == 0xFF ? 0 : filtroAcc << 5);
    const uint8_t nuevoCtrl4 = (ctrl4 & (uint8_t)~0x02) |
                               (IMU_FILTRO_GIRO_ACTIVO ? 0x02 : 0);
    const uint8_t nuevoCtrl6 = (ctrl6 & 0xF8) | IMU_FILTRO_GIRO_CODIGO;
    const uint8_t nuevoCtrlAcc = (odrAcc << 4) | (rangoAcc << 2) |
                                 (filtroAcc == 0xFF ? 0 : 0x02);
    const uint8_t nuevoCtrlGir = (odrGir << 4) | rangoGir;
    if (!escribirRegistroIMU(0x12, 0x44) ||
        !escribirRegistroIMU(0x17, nuevoCtrl8) ||
        !escribirRegistroIMU(0x13, nuevoCtrl4) ||
        !escribirRegistroIMU(0x15, nuevoCtrl6) ||
        !escribirRegistroIMU(0x10, nuevoCtrlAcc) ||
        !escribirRegistroIMU(0x11, nuevoCtrlGir)) return false;

    uint8_t ctrl3 = 0, comprobacion8 = 0, comprobacion4 = 0, comprobacion6 = 0;
    imuInicializada = leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x10, ctrlAcelIMU) &&
                       leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x11, ctrlGiroIMU) &&
                       leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x12, ctrl3) &&
                       leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x17, comprobacion8) &&
                       leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x13, comprobacion4) &&
                       leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x15, comprobacion6) &&
                       ctrlAcelIMU == nuevoCtrlAcc && ctrlGiroIMU == nuevoCtrlGir &&
                       (ctrl3 & 0x44) == 0x44 &&
                       comprobacion8 == nuevoCtrl8 && comprobacion4 == nuevoCtrl4 &&
                       comprobacion6 == nuevoCtrl6;
    imuConectada = imuInicializada;
    Serial.print("[IMU] ID=0x"); Serial.print(idIMU, HEX);
    Serial.print(" CTRL1_XL=0x"); Serial.print(ctrlAcelIMU, HEX);
    Serial.print(" CTRL2_G=0x"); Serial.print(ctrlGiroIMU, HEX);
    Serial.println(imuInicializada ? " OK" : " ERROR");
    return imuInicializada;
  }

  bool leerDatosIMU() {
    // STATUS_REG: bit 0 = dato nuevo acelerometro, bit 1 = dato nuevo giro.
    // No contar muestras repetidas si se reduce el ODR por debajo del loop.
    uint8_t status = 0;
    if (!leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x1E, status)) {
      erroresLecturaIMU++;
      return false;
    }
    if ((status & 0x03) != 0x03) {
      muestrasIMUSinActualizar++;
      return false;
    }
    uint8_t raw[12];
    Wire.beginTransmission(IMU_DIRECCION_I2C);
    Wire.write((uint8_t)0x22); // OUTX_L_G: giro[6] seguido de aceleración[6]
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)IMU_DIRECCION_I2C, (uint8_t)12) != 12) {
      while (Wire.available()) Wire.read();
      erroresLecturaIMU++;
      return false;
    }
    for (uint8_t i = 0; i < 12; i++) raw[i] = (uint8_t)Wire.read();
    for (uint8_t i = 0; i < 3; i++) {
      const int16_t g = (int16_t)((uint16_t)raw[2*i] | ((uint16_t)raw[2*i+1] << 8));
      const int16_t a = (int16_t)((uint16_t)raw[6+2*i] | ((uint16_t)raw[7+2*i] << 8));
      dSt.dato.gyro[i] = g * escalaGiroIMU() + offsetGiro[i];
      dSt.dato.acc[i]  = a * escalaAcelIMU() + offsetAccel[i];
    }
    return true;
  }
////////////////////////////////////

//////////////////////////////////// Incialización de todo
  void iniciarSensores(){
    Wire.begin();
    Wire.setClock(400000);

    //Acel/Giro (ID 0x6C, lectura directa compatible con el test)
      iniciarIMU();
    //Baro
      barInicializado = ps.init();
      barConectado = barInicializado;
      if (barInicializado) {
        ps.enableDefault();
        ps.writeReg(0x10,0b00111010); // Configuración original (comprobar para el modelo LPS exacto)
      }
    //Magne
      magInicializado = mag.begin();
      magConectado = magInicializado;
    //GPS
      UART_GPS.begin(9600);
      
      pinMode(GPS_ON,OUTPUT);
      digitalWrite(GPS_ON,LOW);
      
      //https://forum.arduino.cc/t/configuration-of-chinese-atgm336-gnss/1265640

      // El comando PCAS01,5 cambia el receptor a 115200 baudios.
      // Se envía a 9600 (velocidad habitual de fábrica) y después
      // se cambia la UART del microcontrolador al mismo valor.
      UART_GPS.print("$PCAS01,5*19\r\n");
      UART_GPS.flush();
      delay(50);
      UART_GPS.end();
      UART_GPS.begin(115200);
      UART_GPS.print("$PCAS02,100*1E\r\n"); //Subir a 10Hz
      UART_GPS.print("$PCAS03,1,0,0,0,1,0,0,0*02\r\n"); // GGA + RMC: posicion, fecha y velocidad
      //UART_GPS.print("$PCAS04,3*1A\r\n"); //Usar BDS y GPS
      UART_GPS.print("$PCAS10,0*1C\r\n"); //Resetear automatico si no va
    //Flash  
      //
    //LoRa
      //
    //Buzzer
      pinMode(pin_buzzer, OUTPUT);
  }
////////////////////////////////////



////////////////////////////////////
  // Vacía el UART y actualiza TinyGPSPlus sin hacer espera activa.
  // Se llama una vez por iteración del loop principal.
  void procesarGPS(){
    size_t i = 0;
    while (UART_GPS.available()) {
      const char c = (char)UART_GPS.read();
      if (gps.encode(c)) ultimaTramaGPS = millis(); // NMEA con checksum válido
      if (i < sizeof(recepcionGPS) - 1) recepcionGPS[i++] = c;
    }
    recepcionGPS[i] = '\0';
  }
////////////////////////////////////

//////////////////////////////////// Todo lo necesario para leer los datos de los sensores
  void leerSensores(){
    dSt.dato.estado = estadoDelCohete;

    dSt.dato.tiempo[0] = gps.time.second();
    dSt.dato.tiempo[1] = gps.time.minute();
    dSt.dato.tiempo[2] = gps.time.hour(); // UTC: no sumar horas sin ajustar fecha
    dSt.dato.fecha[0] = gps.date.day();
    dSt.dato.fecha[1] = gps.date.month();
    dSt.dato.fecha[2] = (char)(gps.date.year() - 2000);
    

    dSt.dato.latitud = gps.location.lat();
    dSt.dato.longitud = gps.location.lng();
    dSt.dato.altitud = gps.altitude.meters();
    dSt.dato.velocidad = gps.speed.kmph();
    dSt.dato.gpsValido = gps.location.isValid() && gps.location.age() < 1000 &&
                          gps.altitude.isValid() && gps.hdop.isValid();
    dSt.dato.hdop = gps.hdop.hdop();
    int age = gps.location.age();
    if(age<0 || age>65535){
      age = 65535;
    }
    dSt.dato.tiempoDesdeUltimoFix = age;
    // La referencia geográfica se actualiza solo antes del ascenso.
    if (estadoDelCohete <= esperando_despegue && dSt.dato.gpsValido &&
        isfinite(dSt.dato.latitud) && isfinite(dSt.dato.longitud)) {
      latitudDespegue = dSt.dato.latitud;
      longitudDespegue = dSt.dato.longitud;
      referenciaGPSValida = true;
    }

    //Los ejes de la placa son los de la IMU
    ultimaLecturaIMUValida = imuConectada && leerDatosIMU();
    if (!ultimaLecturaIMUValida) {
      // No permitir que datos de ciclos anteriores confirmen el despegue.
      for (uint8_t i = 0; i < 3; i++) {
        dSt.dato.acc[i] = NAN;
        dSt.dato.gyro[i] = NAN;
      }
    }

    if (magConectado) {
      mag.getEvent(&event);
      dSt.dato.magn[0] = -event.magnetic.y + offsetMagn[0];
      dSt.dato.magn[1] = -event.magnetic.x + offsetMagn[1];
      dSt.dato.magn[2] =  event.magnetic.z + offsetMagn[2];
    }

    if (barConectado) {
      const float presion = ps.readPressureMillibars();
      const uint32_t ahora = millis();
      static bool primeraLectura = true;
      static float presionAnterior = NAN;
      if (isfinite(presion) && presion > 50 && presion < 1200) {
        if (primeraLectura || presion != presionAnterior) ultimaVariacionPresionMs = ahora;
        primeraLectura = false;
        presionAnterior = presion;
      }
      dSt.dato.presion = presion;
      dSt.dato.altitud_barometro = ps.pressureToAltitudeMeters(presion);
      dSt.dato.temperatura = ps.readTemperatureC();
      barLecturaFresca = isfinite(presion) &&
                         (ahora - ultimaVariacionPresionMs <= TIMEOUT_BAR_CONGELADO_MS);
    } else {
      dSt.dato.presion = NAN;
      dSt.dato.altitud_barometro = NAN;
      dSt.dato.temperatura = NAN;
      barLecturaFresca = false;
    }
  }
//////////////////////////////////// 

////////////////////////////////////
  // Se utiliza en setup() para obtener la referencia barométrica inicial.
  float obtenerAlturaBarometrica() {
    return barConectado ? ps.pressureToAltitudeMeters(ps.readPressureMillibars()) : 0.0f;
  }
////////////////////////////////////


//////////////////////////////////// Diagnóstico de los sensores

  // Lectura de un registro de identidad: comprueba errores del bus I2C.
  bool leerIdentificadorI2C(uint8_t direccion, uint8_t registro, uint8_t &id){
    Wire.beginTransmission(direccion);
    Wire.write(registro);
    if(Wire.endTransmission(false) != 0) return false;
    if(Wire.requestFrom(direccion, (uint8_t)1) != 1) return false;
    id = Wire.read();
    return true;
  }

  // Se llama en cada loop. Comprueba I2C a 5 Hz y valores en cada iteración.
  void comprobarEstadoSensores(){
    static uint32_t ultimaComprobacion = 0;
    static bool primeraComprobacion = true;
    uint32_t ahora = millis();

    if (primeraComprobacion || ahora - ultimaComprobacion >= INTERVALO_COMPROBACION_MS) {
      primeraComprobacion = false;
      ultimaComprobacion = ahora;
      uint8_t id = 0;

      imuConectada = imuInicializada &&
                      leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x0F, id) && id == 0x6C &&
                      leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x10, ctrlAcelIMU) &&
                      leerIdentificadorI2C(IMU_DIRECCION_I2C, 0x11, ctrlGiroIMU) &&
                      (ctrlAcelIMU & 0xF0) && (ctrlGiroIMU & 0xF0);
      magConectado = magInicializado && leerIdentificadorI2C(0x1E, 0x4F, id) && id == 0x40;
      barConectado = barInicializado && leerIdentificadorI2C(ps.getAddress(), 0x0F, id) &&
                     (id == 0xBB || id == 0xBD || id == 0xB4);
    }

    // Si falló el arranque o se perdió la IMU, reintentar solo en tierra.
    static uint32_t ultimoReintentoIMU = 0;
    if (estadoDelCohete < ascenso && !imuConectada &&
        ahora - ultimoReintentoIMU >= IMU_REINTENTO_MS) {
      ultimoReintentoIMU = ahora;
      iniciarIMU();
    }

    gpsComunicando = ultimaTramaGPS != 0 && ahora - ultimaTramaGPS < TIMEOUT_GPS_MS;

    bool imuDatosValidos = true, magDatosValidos = true;
    for (int i = 0; i < 3; i++) {
      imuDatosValidos &= isfinite(dSt.dato.acc[i]) && isfinite(dSt.dato.gyro[i]);
      magDatosValidos &= isfinite(dSt.dato.magn[i]);
    }

    bool barDatosValidos = barLecturaFresca && isfinite(dSt.dato.presion) &&
                           dSt.dato.presion > 50.0f && dSt.dato.presion < 1200.0f &&
                           isfinite(dSt.dato.temperatura) && isfinite(dSt.dato.altitud_barometro);
    bool gpsReciente = gpsComunicando && dSt.dato.gpsValido && dSt.dato.tiempoDesdeUltimoFix < 200;
    bool gpsPreciso = gpsReciente && isfinite(dSt.dato.hdop) && dSt.dato.hdop < 5.0f;

    actualizarFlagsSensores(gpsReciente, gpsPreciso,
                           imuConectada && ultimaLecturaIMUValida && imuDatosValidos,
                           magConectado && magDatosValidos,
                           barConectado && barDatosValidos);
  }
////////////////////////////////////

//////////////////////////////////// Lectura de altitud desde el principal
  float obtenerAlturaBarometricaActual(){
    return dSt.dato.altitud_barometro;
  }
////////////////////////////////////

//////////////////////////////////// Historial y datos para el principal
  unionDatos historial[128];
  uint8_t siguienteMuestra = 0;
  uint8_t muestrasDisponibles = 0;

  void actualizarHistorial() {
    historial[siguienteMuestra] = dSt;
    siguienteMuestra = (siguienteMuestra + 1) & 0x7F;
    if (muestrasDisponibles < 128) muestrasDisponibles++;
  }

  bool alturaActualValida() {
    return barConectado && barLecturaFresca &&
           isfinite(dSt.dato.altitud_barometro) &&
           isfinite(dSt.dato.presion) && dSt.dato.presion > 50 && dSt.dato.presion < 1200;
  }

  bool imuActualValida() {
    if (!imuConectada || !ultimaLecturaIMUValida) return false;
    for (int i = 0; i < 3; i++) {
      if (!isfinite(dSt.dato.acc[i]) || !isfinite(dSt.dato.gyro[i])) return false;
    }
    return true;
  }

  bool aceleracionMayor4G() {
    if (!imuConectada || !ultimaLecturaIMUValida) return false;
    float modulo2 = 0;
    for (int i = 0; i < 3; i++) {
      if (!isfinite(dSt.dato.acc[i])) return false;
      modulo2 += dSt.dato.acc[i] * dSt.dato.acc[i];
    }
    return modulo2 > 16.0f;
  }

  void actualizarEstadoEnDatos() { dSt.dato.estado = estadoDelCohete; }
////////////////////////////////////
