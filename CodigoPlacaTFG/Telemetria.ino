/*
 * TELEMETRÍA Y MEMORIAS
 * LoRa: inicialización, compresión y transmisión.
 * SD/Flash: inicialización y almacenamiento.
 * Interfaz para el principal:
 *   iniciarMemorias(), iniciarRadio(), guardarEnMemorias(),
 *   enviarTelemetriaSiToca().
 */
#include <SPI.h>
#include <RadioLib.h>
#include <Adafruit_SPIFlash.h>
#include "flash_devices.h"
#include <SdFat.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

// Pines de SD originales.
#define SD_nCS  8
#define SD_SCK  13
#define SD_MOSI 14
#define SD_MISO 15

#define INTERVALO_SYNC_SD_MS 250  // Confirmar en SD cada 250 ms (no en cada muestra)
#define VELOCIDAD_SPI_SD_MHZ 8    // Velocidad conservadora para empezar las pruebas
#define RADIO_TIMEOUT_MS 5000     // No quedar permanentemente bloqueado por IRQ perdida


// Flags originales: representan el estado de cada componente.
uint8_t flagsSensoresActivos = 0;
#define tfixGPSmenorA200ms 0x01
#define hdopGPSmenorA5m    0x02
#define IMUfuncionando     0x04
#define MAGfuncionando     0x08
#define BARfuncionando     0x10
#define SDfuncionando      0x20
#define FLASHfuncionando   0x40
#define noEnModoTest       0x80

//////////////////////////////////// Protocolo Artemisa III v1: 64 bytes
  // Cabecera[4], version[1], payloadSize[1], secuenciaLE[2],
  // Telemetria[54], CRC16-CCITT-FALSE LE[2].
  struct __attribute__((packed)) TelemetriaArtemisa {
    float acc_x, acc_y, acc_z;
    float pitch, roll, yaw;
    float vel, vel_max;
    float latitud, longitud;
    int16_t altura, altura_max;
    float vel_vert;
    int16_t temperatura, estado, flag;
  };
  static_assert(sizeof(float) == 4 && sizeof(TelemetriaArtemisa) == 54,
                "Telemetria Artemisa debe medir exactamente 54 bytes");
  static_assert(sizeof(int16_t) == 2, "Se requieren enteros de 16 bits");
  #define TAM_TRAMA_LORA 64
  uint8_t datosAmandar[TAM_TRAMA_LORA];
  uint16_t secuenciaTelemetria = 0;
  float velocidadGPSMaxima = 0.0f;
////////////////////////////////////

//////////////////////////////////// Configuración de la flash
  SPIFlash_Device_t const W25Q128JVPIQ {
    .total_size = (1UL << 24),    // 128Mbit, *16Mbyte
    .start_up_time_us = 5000,   	// Time Delay Before Write Instruction 

    .manufacturer_id = 0xEF,		//(MF7-MF0)-17h, (ID15-ID0)-4018h
    .memory_type = 0x40,
    .capacity = 0x18,

    .max_clock_speed_mhz = 133,
    .quad_enable_bit_mask = 0x02, //El mismo
    .has_sector_protection = 1,   //Tiene protección de array de sector
    .supports_fast_read = 1,      
    .supports_qspi = 1,           //Si
    .supports_qspi_writes = 1,    //Tiene la instruccion 32h
    .write_status_register_split = 1, //Tiene 3 registros
    .single_status_byte = 0,      
    .is_fram = 0,                 
  };

  volatile bool FlagLoRaLibre = false;
  
  typedef struct ConfiguracionLoRa {
    float potencia;
    int syncWord;
    int sf;
    float bw;
    int cr;
    int preamble;
  } ConfiguracionLoRa;

  ConfiguracionLoRa maxVelocidad = {20.0, 56, 5, 250.0, 5, 8};
  ConfiguracionLoRa hasta4km = {20.0, 56, 9, 125.0, 5, 8};
  ConfiguracionLoRa maxDistancia = {20.0, 56, 12, 125.0, 7, 12};

  static const uint32_t rfswitch_dio_pins[] = { 
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
  };

  static const Module::RfSwitchMode_t rfswitch_table[] = {
    // mode                  DIO5  DIO6 
    { LR11x0::MODE_STBY,   { LOW,  LOW  } },
    { LR11x0::MODE_RX,     { HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
    END_OF_MODE_TABLE,
  };
////////////////////////////////////



//////////////////////////////////// Globales para la radio, memoria y depuración
  // Radio
  LR1110 radio = new Module(44, 2, 42, 43, SPI);

  // Memoria SD
  SdFat SD;
  SPIClass SD_SPI(NRF_SPIM1, SD_MISO, SD_SCK, SD_MOSI);
  File archivo_salida;
  uint32_t ultimoSyncSD = 0;
  bool datosPendientesSD = false;
  bool errorEscrituraSD = false;

  // Memoria Flash
  Adafruit_FlashTransport_QSPI flashTransport;
  Adafruit_SPIFlash flash(&flashTransport);

  // Depuración
  char buffer[1024];
  int bufferLength = 0;
  uint16_t contMandarMensaje = 0;
  bool radioLista = false;
  bool transmisionLoRaEnCurso = false;
  uint32_t inicioTransmisionMs = 0;
////////////////////////////////////




//////////////////////////////////// Registro binario de la Flash QSPI
  // 128 bytes por muestra: dos muestras por pagina fisica de 256 bytes.
  // NUNCA se borran automaticamente los registros anteriores.
  #define TAM_REGISTRO_FLASH 128UL
  #define MARCA_REGISTRO_FLASH 0x31415644UL  // Identificador de este formato
  #define FIN_REGISTRO_FLASH 0xC0DEC0DEUL

  struct RegistroFlash {
    uint32_t marca;
    uint32_t tiempo_ms;
    uint8_t datos[sizeof(dSt.byte)];
    uint8_t flags;
    uint32_t checksum;
    uint32_t confirmado;                  // Se escribe en una SEGUNDA operacion
    uint32_t vuelo;                       // Nuevo: ID persistente del vuelo
    uint32_t vuelo_inverso;               // Permite detectar ID incompleto
  };
  static_assert(sizeof(RegistroFlash) <= TAM_REGISTRO_FLASH,
                "RegistroFlash excede los 128 bytes reservados");

  bool flashInicializada = false;
  bool flashListaParaGuardar = false;
  uint32_t siguienteDireccionFlash = 0;
  uint32_t ultimoVueloFlash = 0;
  uint32_t vueloFlashActual = 0;
  uint32_t inicioUltimoVueloLegado = UINT32_MAX;
  bool ultimoVueloConIdentificador = false;
  bool sdInicializada = false;

  uint32_t direccionInicioUltimoVuelo = 0;
  uint32_t direccionFinUltimoVuelo = 0;

  uint32_t segundosFlashDisponibles() {
    if (!flashInicializada || !flashListaParaGuardar) return 0;
    return (flash.size() - siguienteDireccionFlash) / TAM_REGISTRO_FLASH / FRECUENCIA_LOOP_HZ;
  }

  bool almacenamientoPreparado() {
    // Puede iniciar sin SD si queda espacio razonable en Flash.
    return (sdInicializada && archivo_salida && !errorEscrituraSD) ||
           segundosFlashDisponibles() >= FLASH_RESERVA_SEGUNDOS;
  }

  bool tieneIDVueloValido(const RegistroFlash &r) {
    return r.vuelo != 0 && r.vuelo != 0xFFFFFFFFUL &&
           r.vuelo_inverso == ~r.vuelo;
  }

  uint32_t checksumFlash(const RegistroFlash &registro) {
    uint32_t hash = 2166136261UL;        // FNV-1a, detecta registros incompletos
    const uint8_t* p = (const uint8_t*)&registro;
    for (size_t i = 0; i < offsetof(RegistroFlash, checksum); i++) {
      hash = (hash ^ p[i]) * 16777619UL;
    }
    return hash;
  }

  bool registroFlashValido(const RegistroFlash &registro) {
    return registro.marca == MARCA_REGISTRO_FLASH &&
           registro.confirmado == FIN_REGISTRO_FLASH &&
           registro.checksum == checksumFlash(registro);
  }

  // El registro es anexado secuencialmente: búsqueda binaria del primer bloque vacío.
  // Una celda escrita parcialmente NO se considera vacía y se deja intacta.
  bool bloqueFlashVacio(uint32_t indice, bool &vacio) {
    uint8_t b[TAM_REGISTRO_FLASH] __attribute__((aligned(4)));
    if (flash.readBuffer(indice * TAM_REGISTRO_FLASH, b, sizeof(b)) != sizeof(b)) return false;
    vacio = true;
    for (size_t i = 0; i < sizeof(b); ++i) if (b[i] != 0xFF) { vacio = false; break; }
    return true;
  }

  bool localizarFinalFlash() {
    const uint32_t cantidad = flash.size() / TAM_REGISTRO_FLASH;
    if (!cantidad) return false;
    bool vacio = false;
    if (!bloqueFlashVacio(0, vacio)) return false;
    if (!vacio) {
      RegistroFlash primero = {};
      if (flash.readBuffer(0, (uint8_t*)&primero, sizeof(primero)) != sizeof(primero) ||
          primero.marca != MARCA_REGISTRO_FLASH) {
        Serial.println("Flash con formato desconocido: no se sobrescribe.");
        return false;
      }
    }
    uint32_t low = 0, high = cantidad;
    while (low < high) {
      uint32_t mid = low + (high-low)/2;
      if (!bloqueFlashVacio(mid, vacio)) return false;
      if (vacio) high = mid; else low = mid + 1;
    }
    siguienteDireccionFlash = low * TAM_REGISTRO_FLASH;
    direccionFinUltimoVuelo = siguienteDireccionFlash;
    ultimoVueloFlash = 0;
    ultimoVueloConIdentificador = false;
    inicioUltimoVueloLegado = UINT32_MAX;
    direccionInicioUltimoVuelo = 0;

    // Inspeccionar desde el final; normalmente solo recorre el último vuelo.
    bool ultimoEncontrado = false;
    uint32_t tiempoAnterior = 0;
    for (uint32_t dir = siguienteDireccionFlash; dir >= TAM_REGISTRO_FLASH; ) {
      dir -= TAM_REGISTRO_FLASH;
      RegistroFlash r = {};
      if (flash.readBuffer(dir, (uint8_t*)&r, sizeof(r)) != sizeof(r)) return false;
      if (!registroFlashValido(r)) continue;
      if (!ultimoEncontrado) {
        ultimoEncontrado = true;
        ultimoVueloConIdentificador = tieneIDVueloValido(r);
        ultimoVueloFlash = ultimoVueloConIdentificador ? r.vuelo : 0;
        direccionInicioUltimoVuelo = dir;
        if (!ultimoVueloConIdentificador) inicioUltimoVueloLegado = dir;
        unionDatos muestra = {};
        memcpy(muestra.byte, r.datos, sizeof(muestra.byte));
        tiempoAnterior = r.tiempo_ms;
      } else if (ultimoVueloConIdentificador) {
        if (!tieneIDVueloValido(r) || r.vuelo != ultimoVueloFlash) break;
        direccionInicioUltimoVuelo = dir;
      } else {
        if (tieneIDVueloValido(r)) break;
        unionDatos muestra = {};
        memcpy(muestra.byte, r.datos, sizeof(muestra.byte));
        // El comienzo de una nueva sesión se reconoce por el reinicio de millis().
        if (r.tiempo_ms > tiempoAnterior) break;
        direccionInicioUltimoVuelo = inicioUltimoVueloLegado = dir;
        tiempoAnterior = r.tiempo_ms;
      }
    }
    return true;
  }

  bool guardarMuestraFlash() {
    if (!flashListaParaGuardar) return false;
    if (siguienteDireccionFlash + TAM_REGISTRO_FLASH > flash.size()) {
      flashListaParaGuardar = false;
      flagsSensoresActivos &= ~FLASHfuncionando;
      return false;
    }

    // Un ID nuevo para cada despegue. El anterior se recupera al arrancar.
    if (!vueloFlashActual) {
      if (ultimoVueloFlash == UINT32_MAX) return false;
      vueloFlashActual = ultimoVueloFlash + 1;
      ultimoVueloFlash = vueloFlashActual;
    }

    RegistroFlash registro = {};
    registro.marca = MARCA_REGISTRO_FLASH;
    registro.tiempo_ms = millis();
    memcpy(registro.datos, dSt.byte, sizeof(dSt.byte));
    registro.flags = flagsSensoresActivos;
    registro.checksum = checksumFlash(registro);
    registro.confirmado = 0xFFFFFFFFUL; // Todavía no confirmado
    registro.vuelo = vueloFlashActual;
    registro.vuelo_inverso = ~registro.vuelo;

    // No sobrescribir una celda parcial o datos que no hayan sido borrados.
    bool celdaLibre = false;
    if (!bloqueFlashVacio(siguienteDireccionFlash / TAM_REGISTRO_FLASH, celdaLibre) || !celdaLibre) {
      flashListaParaGuardar = false;
      flagsSensoresActivos &= ~FLASHfuncionando;
      return false;
    }

    // Dos fases: datos e ID primero, confirmación AL FINAL.
    // El formato anterior sigue siendo legible: el checksum no cambia.
    const uint32_t dir = siguienteDireccionFlash;
    if (flash.writeBuffer(dir, (const uint8_t*)&registro, sizeof(registro)) != sizeof(registro)) {
      flashListaParaGuardar = false;
      flagsSensoresActivos &= ~FLASHfuncionando;
      return false;
    }
    const uint32_t confirmado = FIN_REGISTRO_FLASH;
    if (flash.writeBuffer(dir + offsetof(RegistroFlash, confirmado),
                          (const uint8_t*)&confirmado, sizeof(confirmado)) != sizeof(confirmado)) {
      flashListaParaGuardar = false;      // No reintentar sobre una celda a medio escribir
      flagsSensoresActivos &= ~FLASHfuncionando;
      return false;
    }
    flash.waitUntilReady();               // Esperar a que termine la programacion real
    siguienteDireccionFlash += TAM_REGISTRO_FLASH;
    return true;
  }
////////////////////////////////////

  const char *cabeceraCSV = "tiempo_ms\tvuelo\testado\tflags\tdia\tmes\tano\thora_utc\tminuto\tsegundo\tacc_x_g\tacc_y_g\tacc_z_g\tgiro_x_dps\tgiro_y_dps\tgiro_z_dps\tmag_x\tmag_y\tmag_z\tlat\tlon\talt_gps_m\tvel_gps_kmh\thdop\tedad_fix_ms\tpresion_hpa\ttemperatura_C\talt_bar_m\n";

//////////////////////////////////// Funciones de las flash
  char iniciarMemorias(){
    char out = 0;
    //Flash
      flashInicializada = flash.begin(&W25Q128JVPIQ, 1);
      if(!flashInicializada || !localizarFinalFlash()){
        Serial.println("Error iniciando el registro Flash");
        out |= 0x1;
      }else{
        flashListaParaGuardar = siguienteDireccionFlash + TAM_REGISTRO_FLASH <= flash.size();
        if (flashListaParaGuardar) {
          flagsSensoresActivos |= FLASHfuncionando;
          Serial.print("Flash: siguiente direccion = ");
          Serial.println(siguienteDireccionFlash);
          Serial.print("Flash: segundos aprox. restantes a 100 Hz = ");
          Serial.println(segundosFlashDisponibles());
        }else{
          Serial.println("Flash llena. Exportar los datos antes de borrarla.");
          out |= 0x1;
        }
      }
    //SD: el objeto SD usa SD_SPI, NO el SPI del LoRa
      pinMode(SD_nCS, OUTPUT);
      digitalWrite(SD_nCS, HIGH);
      SD_SPI.begin();
      sdInicializada = SD.begin(SdSpiConfig(SD_nCS, DEDICATED_SPI,
                                  SD_SCK_MHZ(VELOCIDAD_SPI_SD_MHZ), &SD_SPI));
      if(!sdInicializada){
        Serial.println("Error al inicializar la tarjeta SD");
        out |= 0x2;
      }else{
        // No borrar el archivo anterior. Crear uno nuevo en cada arranque.
        char nombre[13];
        for(unsigned int i = 0; i < 1000; i++){
          snprintf(nombre, sizeof(nombre), "VUELO%03u.CSV", i);
          if(!SD.exists(nombre)){
            archivo_salida = SD.open(nombre, FILE_WRITE);
            break;
          }
        }
        if(!archivo_salida){
          Serial.println("Error creando el archivo de registro (o no hay nombres libres)");
          out |= 0x4;
        }else{
          Serial.print("Guardando en: ");
          Serial.println(nombre);
          if (archivo_salida.write((const uint8_t*)cabeceraCSV, strlen(cabeceraCSV)) != strlen(cabeceraCSV) ||
              !archivo_salida.sync()) {
            errorEscrituraSD = true;
            out |= 0x4;
          } else flagsSensoresActivos |= SDfuncionando;
          ultimoSyncSD = millis();
        }
      }
    return out;
  }
////////////////////////////////////



//////////////////////////////////// Control de la radio
  void setFlag(void) {
    // we sent a packet, set the flag
    FlagLoRaLibre = true;
  }

  int iniciarRadio(){
    SPI.setPins(47, 45, 46); // Pines originales del LoRa
    SPI.begin();
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
    int state = radio.begin(868.0, hasta4km.bw, hasta4km.sf, hasta4km.cr,
                            hasta4km.syncWord, hasta4km.potencia, hasta4km.preamble, 1.6);
    radioLista = (state == RADIOLIB_ERR_NONE);
    FlagLoRaLibre = false;
    if (radioLista) radio.setPacketSentAction(setFlag);
    return state;
  }

  int cambiarConfiguracionLora(ConfiguracionLoRa* target){
    if (transmisionLoRaEnCurso) return -1;
    int state = radio.begin(868.0, target->bw, target->sf, target->cr,
                             target->syncWord, target->potencia, target->preamble, 1.6);
    radioLista = (state == RADIOLIB_ERR_NONE);
    if (radioLista) radio.setPacketSentAction(setFlag);
    return state;
  }

  int transmitirDatos(uint8_t* arrayAmandar, size_t longitudArray){
    comprimirParaTransmision();
    return radio.startTransmit(arrayAmandar, longitudArray);
  }
//////////////////////////////////// Protocolo de telemetría Artemisa III
  // CRC16-CCITT-FALSE, igual que Comunicaciones.cpp del emisor de referencia.
  uint16_t crc16Trama(const uint8_t *datos, size_t longitud) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < longitud; ++i) {
      crc ^= (uint16_t)datos[i] << 8;
      for (uint8_t b = 0; b < 8; ++b)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                             : (uint16_t)(crc << 1);
    }
    return crc;
  }

  // Evitar conversiones de NaN, infinitos y desbordamientos a int16_t.
  int16_t entero16Seguro(float valor) {
    if (!isfinite(valor)) return 0;
    if (valor > 32767.0f) return 32767;
    if (valor < -32768.0f) return -32768;
    return (int16_t)valor; // El emisor de referencia trunca, no redondea.
  }

  float realSeguro(float valor) {
    return isfinite(valor) ? valor : 0.0f;
  }

  // La placa original distingue seis estados; Artemisa III utiliza cuatro.
  int16_t estadoCompatibleArtemisa() {
    switch (estadoDelCohete) {
      case cohete_desarmado:
      case esperando_despegue:    return 0;  // En tierra / espera
      case ascenso:               return 1;  // Ascenso
      case drogue_desplegado:
      case paracaidas_desplegado: return 2;  // Descenso
      case aterrizado:            return 3;  // Aterrizaje
      default:                    return 0;
    }
  }

  void comprimirParaTransmision() {
    TelemetriaArtemisa t = {};
    const auto &d = dSt.dato;

    // Aceleración [g] y orientación [grados]. Orientación geométrica
    // como en Sensores.cpp de Artemisa III (no es una fusión AHRS).
    t.acc_x = realSeguro(d.acc[0]);
    t.acc_y = realSeguro(d.acc[1]);
    t.acc_z = realSeguro(d.acc[2]);
    if (imuActualValida()) {
      t.pitch = atan2f(-t.acc_x, sqrtf(t.acc_y*t.acc_y + t.acc_z*t.acc_z)) * (180.0f / PI);
      t.roll = atan2f(t.acc_y, t.acc_z) * (180.0f / PI);
    }
    if (magConectado && isfinite(d.magn[0]) && isfinite(d.magn[1])) {
      t.yaw = atan2f(d.magn[1], d.magn[0]) * (180.0f / PI);
      if (t.yaw < 0.0f) t.yaw += 360.0f;
    }

    // La placa guarda velocidad GPS en km/h; Artemisa la envía en m/s.
    if (d.gpsValido && isfinite(d.velocidad) && d.velocidad >= 0.0f) {
      t.vel = d.velocidad / 3.6f;
      if (t.vel > velocidadGPSMaxima) velocidadGPSMaxima = t.vel;
    }
    t.vel_max = velocidadGPSMaxima;
    if (d.gpsValido && isfinite(d.latitud) && isfinite(d.longitud)) {
      t.latitud = d.latitud;
      t.longitud = d.longitud;
    }

    // Artemisa trabaja con altura sobre el punto de lanzamiento.
    if (alturaActualValida()) {
      const float relativa = d.altitud_barometro - alturaDespegue;
      if (isfinite(relativa)) {
        t.altura = entero16Seguro(relativa);
      }
    }
    // Máximo calculado en cada ciclo del loop (100 Hz), relativo al despegue.
    // Sigue disponible incluso después del apogeo y del aterrizaje.
    if (estadoDelCohete >= ascenso && alturaMaxima > -99999.0f &&
        isfinite(alturaMaxima) && isfinite(alturaDespegue)) {
      t.altura_max = entero16Seguro(fmaxf(0.0f, alturaMaxima - alturaDespegue));
    }
    t.vel_vert = velocidadValida ? realSeguro(velocidadVertical) : 0.0f;
    t.temperatura = entero16Seguro(d.temperatura);
    t.estado = (int16_t)estadoDelCohete;

    // En esta aviónica los paracaídas son mecánicos. No existe medida
    // de continuidad: -1 indica «no disponible», NO «continuidad OK».
    t.flag = -1;

    // Cabecera y layout binario exactos del emisor Artemisa III (v1).
    static const uint8_t cabecera[4] = {0xA5, 0x5A, 0xC3, 0x3C};
    memcpy(datosAmandar, cabecera, sizeof(cabecera));
    datosAmandar[4] = 1;
    datosAmandar[5] = (uint8_t)sizeof(t);
    datosAmandar[6] = (uint8_t)(secuenciaTelemetria & 0xFF);
    datosAmandar[7] = (uint8_t)(secuenciaTelemetria >> 8);
    memcpy(datosAmandar + 8, &t, sizeof(t));
    const uint16_t crc = crc16Trama(datosAmandar, TAM_TRAMA_LORA - 2);
    datosAmandar[62] = (uint8_t)crc;
    datosAmandar[63] = (uint8_t)(crc >> 8);
    secuenciaTelemetria++;
  }

  void enviarTelemetriaSiToca() {
    if (!radioLista) return;
    // Terminar TX antes de enviar de nuevo (desde el loop, nunca desde la ISR).
    if (transmisionLoRaEnCurso) {
      if (FlagLoRaLibre || millis() - inicioTransmisionMs >= RADIO_TIMEOUT_MS) {
        int resultado = radio.finishTransmit();
        transmisionLoRaEnCurso = false;
        FlagLoRaLibre = false;
        if (resultado != RADIOLIB_ERR_NONE) {
          radioLista = false; // No encadenar transmisiones si radio no quedó lista.
          return;
        }
      } else return;
    }
    if (++contMandarMensaje < ciclosAsaltarTransmision) return;
    contMandarMensaje = 0;
    const int estado = transmitirDatos(datosAmandar, sizeof(datosAmandar));
    if (estado == RADIOLIB_ERR_NONE) {
      transmisionLoRaEnCurso = true;
      inicioTransmisionMs = millis();
    }
  }

//////////////////////////////////// Guardado en memoria

  int datosAcsv(char* destino, size_t capacidad, const unionDatos &dato,
                uint32_t instante, uint8_t flags, uint32_t vuelo) {
    const auto &d = dato.dato;
    int n = snprintf(destino, capacidad,
        "%lu\t%lu\t%d\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%.5f\t%.5f\t%.5f\t%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.7f\t%.7f\t%.2f\t%.2f\t%.2f\t%u\t%.3f\t%.2f\t%.3f\n",
        (unsigned long)instante, (unsigned long)vuelo, (int)d.estado, (unsigned int)flags,
        (unsigned char)d.fecha[0], (unsigned char)d.fecha[1], (unsigned char)d.fecha[2],
        (unsigned char)d.tiempo[2], (unsigned char)d.tiempo[1], (unsigned char)d.tiempo[0],
        d.acc[0], d.acc[1], d.acc[2], d.gyro[0], d.gyro[1], d.gyro[2],
        d.magn[0], d.magn[1], d.magn[2], d.latitud, d.longitud,
        d.altitud, d.velocidad, d.hdop, d.tiempoDesdeUltimoFix,
        d.presion, d.temperatura, d.altitud_barometro);
    return n >= 0 && (size_t)n < capacidad ? n : -1;
  }

  // Archivo determinista por vuelo. Un CSV completo no se reexporta al reiniciar.
  // TMP permite descartar una exportación interrumpida sin confundirse con un CSV bueno.
  bool exportarUltimoVueloASD() {
    if (!sdInicializada || !flashInicializada || direccionFinUltimoVuelo <= direccionInicioUltimoVuelo)
      return false;
    char csv[13], tmp[13];
    if (ultimoVueloConIdentificador) {
      if (ultimoVueloFlash > 9999999UL) {
        Serial.println("[FLASH] ID de vuelo mayor de 7 cifras; exportación no disponible.");
        return false;
      }
      snprintf(csv, sizeof(csv), "L%07lu.CSV", (unsigned long)ultimoVueloFlash);
      snprintf(tmp, sizeof(tmp), "L%07lu.TMP", (unsigned long)ultimoVueloFlash);
    } else {
      strcpy(csv, "LEGACY.CSV");
      strcpy(tmp, "LEGACY.TMP");
    }
    if (SD.exists(csv)) {
      Serial.print("[FLASH] Último vuelo ya exportado: "); Serial.println(csv);
      return true;
    }
    if (SD.exists(tmp) && !SD.remove(tmp)) return false;
    File destino = SD.open(tmp, FILE_WRITE);
    if (!destino) return false;
    bool correcto = destino.write((const uint8_t*)cabeceraCSV, strlen(cabeceraCSV)) == strlen(cabeceraCSV);
    unionDatos muestra = {};
    uint32_t recuperados = 0, descartados = 0;
    RegistroFlash registro = {};
    for (uint32_t dir = direccionInicioUltimoVuelo;
         correcto && dir < direccionFinUltimoVuelo; dir += TAM_REGISTRO_FLASH) {
      if (flash.readBuffer(dir, (uint8_t*)&registro, sizeof(registro)) != sizeof(registro)) {
        correcto = false; break;
      }
      if (!registroFlashValido(registro) ||
          (ultimoVueloConIdentificador &&
           (!tieneIDVueloValido(registro) || registro.vuelo != ultimoVueloFlash))) {
        descartados++; continue;
      }
      memcpy(muestra.byte, registro.datos, sizeof(muestra.byte));
      const int n = datosAcsv(buffer, sizeof(buffer), muestra, registro.tiempo_ms,
                              registro.flags, ultimoVueloConIdentificador ? registro.vuelo : 0);
      if (n < 0 || destino.write((const uint8_t*)buffer, n) != (size_t)n) correcto = false;
      else recuperados++;
    }
    if (!destino.sync()) correcto = false;
    destino.close();
    // Solo un CSV con todas sus filas y sincronización confirmada se publica como .CSV
    if (correcto && recuperados) correcto = SD.rename(tmp, csv);
    Serial.print("[FLASH] Exportación "); Serial.print(csv);
    Serial.print(" muestras="); Serial.print(recuperados);
    Serial.print(" incompletas="); Serial.print(descartados);
    Serial.print(" resultado="); Serial.println(correcto ? "OK" : "ERROR (TMP recuperable)");
    return correcto && recuperados > 0;
  }

  char guardarEnMemorias(bool enFlash, bool enSD){
    char out = 0;
    if (enFlash && flashListaParaGuardar && !guardarMuestraFlash()) out |= 0x1;
    if (!enSD) return out;
    if (!archivo_salida || errorEscrituraSD) {
      flagsSensoresActivos &= ~SDfuncionando;
      return out | 0x2;
    }
    const int n = datosAcsv(buffer, sizeof(buffer), dSt, millis(), flagsSensoresActivos, vueloFlashActual);
    if (n < 0 || archivo_salida.write((const uint8_t*)buffer, n) != (size_t)n) {
      errorEscrituraSD = true;
      flagsSensoresActivos &= ~SDfuncionando;
      return out | 0x2;
    }
    datosPendientesSD = true;
    if (millis() - ultimoSyncSD >= INTERVALO_SYNC_SD_MS) {
      ultimoSyncSD = millis();
      if (!archivo_salida.sync()) {
        errorEscrituraSD = true;
        flagsSensoresActivos &= ~SDfuncionando;
        out |= 0x2;
      } else datosPendientesSD = false;
    }
    return out;
  }

  bool finalizarGuardado() {
    if (!archivo_salida) return false;
    bool correcto = archivo_salida.sync();
    archivo_salida.close();
    datosPendientesSD = false;
    if (!correcto) flagsSensoresActivos &= ~SDfuncionando;
    return correcto;
  }
////////////////////////////////////

//////////////////////////////////// Diagnóstico: flags de salud y mensaje por Serial
  // Los bits originales de LoRa se conservan, sin cambiar su posición.
  void actualizarFlagsSensores(bool gpsReciente, bool gpsPreciso, bool imuOK, bool magOK, bool barOK){
    // Los estados de SD y Flash se actualizan en las funciones de memoria.
    uint8_t flags = flagsSensoresActivos & (SDfuncionando | FLASHfuncionando);
    if(gpsReciente) flags |= tfixGPSmenorA200ms;
    if(gpsPreciso) flags |= hdopGPSmenorA5m;
    if(imuOK)      flags |= IMUfuncionando;
    if(magOK)      flags |= MAGfuncionando;
    if(barOK)      flags |= BARfuncionando;
    #if !modoTest
      flags |= noEnModoTest;
    #endif
    flagsSensoresActivos = flags;
  }
