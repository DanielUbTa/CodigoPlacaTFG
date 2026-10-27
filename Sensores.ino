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
#include <LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>

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
LSM6DS3 myIMU(I2C_MODE, 0x6A);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
sensors_event_t event;
LPS ps;
char recepcionGPS[512];

// Offsets originales, en las mismas unidades de cada librería.
float offsetAccel[] = {-0.004, 0.0065, -0.0145};
float offsetGiro[]  = {-0.52, 0.64, 0.03};
float offsetMagn[]  = {-78.0, -92.0, +15.5};

// Utilizados solo si se activa la opción de depuración de medias.
float gananciaParaLaMedia = 0.02f;
float mediaAcc[3], mediaGiro[3], mediaMagn[3];

//////////////////////////////////// Incialización de todo
  void iniciarSensores(){
    Wire.setClock(400000);

    //Acel/Giro
      myIMU.begin();
    //Baro
      ps.init();
      ps.enableDefault();
      ps.writeReg(0x10,0b00111010); // 0 ___ 0111 para 100Hz ___ 010 para hacer media de 16 valores
    //Magne
      mag.begin();
    //GPS
      UART_GPS.begin(9600);
      
      pinMode(GPS_ON,OUTPUT);
      digitalWrite(GPS_ON,LOW);
      
      //https://forum.arduino.cc/t/configuration-of-chinese-atgm336-gnss/1265640

      // REVISAR: el cambio posterior a 115200 no se acompaña de
      // un comando que configure expresamente el receptor a 115200.
      // No se altera aquí para mantener el comportamiento original.
      //No cambiar el baud rate
      UART_GPS.print("$PCAS01,1*1D\r\n"); //Cambiar baud rate a 9600
      //UART_GPS.begin(9600);
      //UART_GPS.print("$PCAS01,2*1E\r\n"); //Cambiar baud rate a 19200
      //UART_GPS.begin(19200);

      //UART_GPS.print("$PCAS01,5*19\r\n"); //Cambiar baud rate a 115200
      delay(10);
      //UART_GPS.end();
      UART_GPS.begin(115200);
      UART_GPS.print("$PCAS02,100*1E\r\n"); //Subir a 10Hz
      UART_GPS.print("$PCAS03,1,0,0,0,0,0,0,0*03\r\n"); //Recivir solo mensajes de GGA
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
      gps.encode(c);
      if (i < sizeof(recepcionGPS) - 1) recepcionGPS[i++] = c;
    }
    recepcionGPS[i] = '\0';
  }
////////////////////////////////////

//////////////////////////////////// Todo lo necesario para leer los datos de 
  void leerSensores(){
    dSt.dato.estado = estadoDelCohete;

    dSt.dato.tiempo[0] = gps.time.second();
    dSt.dato.tiempo[1] = gps.time.minute();
    dSt.dato.tiempo[2] = (gps.time.hour()+1)%24; //Ajustar para hora en España
    dSt.dato.fecha[0] = gps.date.day();
    dSt.dato.fecha[1] = gps.date.month();
    dSt.dato.fecha[2] = (char)(gps.date.year() - 2000);
    

    dSt.dato.latitud = gps.location.lat();
    dSt.dato.longitud = gps.location.lng();
    dSt.dato.altitud = gps.altitude.meters();
    dSt.dato.velocidad = gps.speed.kmph();
    dSt.dato.gpsValido = gps.location.isValid() && gps.altitude.isValid() && gps.hdop.isValid();
    dSt.dato.hdop = gps.hdop.hdop();
    int age = gps.location.age();
    if(age<0 || age>65535){
      age = 65535;
    }
    dSt.dato.tiempoDesdeUltimoFix = age;

    //Los ejes de la placa son los de la IMU
    dSt.dato.acc[0] = myIMU.readFloatAccelX() + offsetAccel[0];
    dSt.dato.acc[1] = myIMU.readFloatAccelY() + offsetAccel[1];
    dSt.dato.acc[2] = myIMU.readFloatAccelZ() + offsetAccel[2];
    dSt.dato.gyro[0] = myIMU.readFloatGyroX() + offsetGiro[0];
    dSt.dato.gyro[1] = myIMU.readFloatGyroY() + offsetGiro[1];
    dSt.dato.gyro[2] = myIMU.readFloatGyroZ() + offsetGiro[2];

    mag.getEvent(&event);
    dSt.dato.magn[0] = -event.magnetic.y + offsetMagn[0];
    dSt.dato.magn[1] = -event.magnetic.x + offsetMagn[1];
    dSt.dato.magn[2] =  event.magnetic.z + offsetMagn[2];

    dSt.dato.presion = ps.readPressureMillibars();
    dSt.dato.altitud_barometro = ps.pressureToAltitudeMeters(dSt.dato.presion);
    dSt.dato.temperatura = ps.readTemperatureC();
  }
//////////////////////////////////// 

////////////////////////////////////
  // Se utiliza en setup() para obtener la referencia barométrica inicial.
  float obtenerAlturaBarometrica() {
    return ps.pressureToAltitudeMeters(ps.readPressureMillibars());
  }
////////////////////////////////////
