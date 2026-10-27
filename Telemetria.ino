/*
 * TELEMETRÍA Y MEMORIAS
 * LoRa: inicialización, compresión y transmisión.
 * SD/Flash: inicialización y almacenamiento; depuración por Serial.
 * Interfaz para el principal:
 *   iniciarMemorias(), iniciarRadio(), guardarEnMemorias(),
 *   enviarTelemetriaSiToca(), mostrarDebug().
 */
#include <SPI.h>
#include <RadioLib.h>
#include <Adafruit_SPIFlash.h>
#include "flash_devices.h"
#include <SdFat.h>
#include <math.h>

// Pines de SD originales.
#define SD_nCS  8
#define SD_SCK  13
#define SD_MOSI 14
#define SD_MISO 15

#define cuantosSaltarseDebug 1
#define debugPrintDatos true
#define debugDatosBytes false
#define debugParaSerialPlotter false
#define debugTiempoComputo true
#define debugGPS false
#define debugRecepcionGPS false
#define debuggearValorMedioSensores false

// Flags originales. El programa original todavía no los actualiza.
char flagsSensoresActivos = 0;
#define tfixGPSmenorA200ms 0x01
#define hdopGPSmenorA5m    0x02
#define IMUfuncionando     0x04
#define MAGfuncionando     0x08
#define BARfuncionando     0x10
#define SDfuncionando      0x20
#define FLASHfuncionando   0x40
#define noEnModoTest       0x80

char datosAmandar[14];  // Formato binario original, 14 bytes.

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

  // Memoria Flash
  Adafruit_FlashTransport_QSPI flashTransport;
  Adafruit_SPIFlash flash(&flashTransport);

  // Depuración
  char buffer[1024];
  int bufferLength = 0;
  uint16_t contMandarMensaje = 0;
////////////////////////////////////



//////////////////////////////////// Funciones de las flash
  char iniciarMemorias(){
    char out = 0;
    bool SDfuncional = true;
    //Flash
      if(!flash.begin(&W25Q128JVPIQ, 1)){
        Serial.println("Error iniciando la flash");
        out |= 0x1;
      };
    //SD  
      SD_SPI.begin();
      pinMode(SD_nCS,OUTPUT);
      //delay(10); //Lo meto porque si, no lo recomiendan en ningún lado
      //SD_SPI.setPins(SD_MISO, SD_SCK, SD_MOSI);
      if (!SD.begin(SD_nCS)) {
        Serial.println("Error al inicializar la tarjeta SD");
        out |= 0x2;
        SDfuncional = false;
      }
    //Archivo de la SD
        if(SDfuncional){
          SD.remove("salida.txt");
          archivo_salida = SD.open("salida.txt", FILE_WRITE);
          if(!archivo_salida){
            Serial.println("Error al crear el archivo de salida");
            out |= 0x4;
          }
          #if modoTest
            if(archivo_salida){
              archivo_salida.close();
              archivo_salida = SD.open("test.txt", FILE_WRITE);
              archivo_salida.println("Tonto el que lo lea");
              archivo_salida.close();
              Serial.println("Archivo de test escrito y cerrado");
              archivo_salida = SD.open("salida.txt", FILE_WRITE);
            }
          #endif
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
    int state = radio.begin(868.0, hasta4km.bw, hasta4km.sf, hasta4km.cr, hasta4km.syncWord, hasta4km.potencia, hasta4km.preamble, 1.6);
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
    FlagLoRaLibre = (state == RADIOLIB_ERR_NONE);
    radio.setPacketSentAction(setFlag);
    return state;
  }

  int cambiarConfiguracionLora(ConfiguracionLoRa* target){
    int state = radio.begin(868.0, target->bw, target->sf, target->cr, target->syncWord, target->potencia, target->preamble, 1.6);
    return state;
  }

  int transmitirDatos(char* arrayAmandar, int longitudArray){
    comprimirParaTransmision();
    int state = radio.startTransmit(arrayAmandar,longitudArray);
    return state;
  }
//////////////////////////////////// Funciones extra para la radio
  void comprimirParaTransmision(){/*
      AcelVertical[ 7:0]        lectura del acelerómetro para sensibilidad de +-16G
      AcelVertical[15:8]
      xMagn[ 7:0]               multiplicar por 20
      yMagn[ 3:0],xMagn[11:8]
      yMagn[11:4]
      zMagn[ 7:0]
      Altura[ 3:0],zMagn[11:8]  para la altura el cohete solo está previsto para llegar a 3km de altura, cabe en 12 bits
      Altura[11:4]
      metrosNorte[ 7:0]         no debería irse más de 1km a la redonda, aún así aquí caben +-2048 metros
      metrosEste[ 3:0],metrosNorte[11:8]
      metrosEste[11:4]          metros desde el punto de despegue
      temp[ 7:0]                (temperatura-20.0)*10.0
      estado[4:0],temp[ 10:8]   hasta 32 estados
      flags[ 7:0]               la variable "flagsSensoresActivos"
      */
    int lecturaAcelerometro = (int)(dSt.dato.acc[0] * (65536.0/16.0));
    datosAmandar[0] = lecturaAcelerometro & 0xFF;
    datosAmandar[1] = (lecturaAcelerometro>>8) & 0xFF;

    int xMagn = (int)(dSt.dato.magn[0] * 20);
    int yMagn = (int)(dSt.dato.magn[1] * 20);
    int zMagn = (int)(dSt.dato.magn[2] * 20);
    datosAmandar[2] = xMagn & 0xFF;
    datosAmandar[3] = (xMagn>>8) & 0xF | (yMagn<<4) & 0xF0;
    datosAmandar[4] = (yMagn>>4) & 0xFF;
    datosAmandar[5] = zMagn & 0xFF;

    int altura = (int)(dSt.dato.altitud_barometro) - alturaDespegue;
    datosAmandar[6] = (zMagn>>8) & 0xF | (altura<<4) & 0xF0;
    datosAmandar[7] = (altura>>4) & 0xFF;

    int metrosNorte = (int)((dSt.dato.latitud-latitudDespegue)*40075000/360.0);
    int metrosEste = (int)((dSt.dato.longitud-longitudDespegue)*40075000/360.0) * cos(dSt.dato.latitud*2.0*M_PI/360.0); //Para la distorsión cerca de los polos
    datosAmandar[8] = metrosNorte & 0xFF;
    datosAmandar[9] = (metrosNorte>>8) & 0xF | (metrosEste<<4) & 0xF0;
    datosAmandar[10] = (metrosEste>>4) & 0xFF;

    int temp = (int)(dSt.dato.temperatura*10.0-200.0);
    datosAmandar[11] = temp & 0xFF;

    datosAmandar[12] = (temp>>8) & 0x7 | (estadoDelCohete<<3) & 0xF8;

    datosAmandar[13] = flagsSensoresActivos;
  }

  // Gestiona por sí misma el contador; el principal no necesita conocerlo.
  void enviarTelemetriaSiToca() {
    contMandarMensaje++;
    if (contMandarMensaje >= ciclosAsaltarTransmision && FlagLoRaLibre) {
      contMandarMensaje = 0;
      FlagLoRaLibre = false;
      // transmitirDatos() ya llama a comprimirParaTransmision().
      const int error = transmitirDatos(datosAmandar, sizeof(datosAmandar));
      if (error != RADIOLIB_ERR_NONE) FlagLoRaLibre = true;
    }
  }
////////////////////////////////////



//////////////////////////////////// Guardado en memoria
  int datosAcsv(char* destino){
    snprintf(destino, sizeof(buffer), "%2d\t%2d\t20%2d\t%2d\t%2d\t%2d\t% 2.6f\t% 2.6f\t% 2.6f\t% 6.2f\t% 6.2f\t% 6.2f\t% 6.2f\t% 6.2f\t% 6.2f\t%f\t%f\t%.2f\t%.3f\t%.2f\t%4d\t%4.4f\t% 3.2f\t% 4.2f\n",
          dSt.dato.fecha[0],dSt.dato.fecha[1],dSt.dato.fecha[2],dSt.dato.tiempo[2],dSt.dato.tiempo[1],dSt.dato.tiempo[0],
          dSt.dato.acc[0],dSt.dato.acc[1],dSt.dato.acc[2],dSt.dato.gyro[0],dSt.dato.gyro[1],dSt.dato.gyro[2],dSt.dato.magn[0],dSt.dato.magn[1],dSt.dato.magn[2],
          /*(dSt.dato.gpsValido)? 1:0,*/ dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad,dSt.dato.hdop,dSt.dato.tiempoDesdeUltimoFix,
          dSt.dato.presion,dSt.dato.temperatura,dSt.dato.altitud_barometro);
    return strlen(destino);
  }

  char guardarEnMemorias(bool enFlash, bool enSD){
    char out = 0;// Si no es 0 ha habido un error
    int longitud = datosAcsv(buffer);
    bufferLength = longitud;
    if(enFlash){
      //Acabar
      /*
      flash.eraseChip();
      flash.eraseSector(0);
      flash.waitUntilReady();
      flash.writeBuffer(addr, bufwrite, sizeof(bufwrite));
      flash.readBuffer(addr, bufread, sizeof(bufread));
      */

      out |= 0x1;
    }
    if(enSD){
      if(archivo_salida){
        int longitudSD = archivo_salida.print(buffer);
        archivo_salida.flush();
        if(longitudSD != longitud){
          out |= 0x2;
        }
      }else{
        out |= 0x2;
      }
    }
    //Serial.print(buffer);

    return out;
  }
////////////////////////////////////



//////////////////////////////////// Depuración
  void debuggearTodo(){
    cont++;
    if(cont == cuantosSaltarseDebug){

      #if debugRecepcionGPS
        Serial.print(recepcionGPS);
      #endif

      #if debugPrintDatos
        sprintf(buffer, "%2d-%2d-20%2d %2d:%2d:%2d ACEL:% 2.6f,% 2.6f,% 2.6f GIRO:% 6.2f,% 6.2f,% 6.2f MAG:% 6.2f,% 6.2f,% 6.2f GPS:%f,%f,h:%.2f,v:%.3f,hdop:%.2f,fix:%4d BAR:%4.4f,% 3.2f,h:% 4.2f",
          dSt.dato.fecha[0],dSt.dato.fecha[1],dSt.dato.fecha[2],dSt.dato.tiempo[2],dSt.dato.tiempo[1],dSt.dato.tiempo[0],
          dSt.dato.acc[0],dSt.dato.acc[1],dSt.dato.acc[2],dSt.dato.gyro[0],dSt.dato.gyro[1],dSt.dato.gyro[2],dSt.dato.magn[0],dSt.dato.magn[1],dSt.dato.magn[2],
          /*(dSt.dato.gpsValido)? 1:0,*/ dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad,dSt.dato.hdop,dSt.dato.tiempoDesdeUltimoFix,
          dSt.dato.presion,dSt.dato.temperatura,dSt.dato.altitud_barometro);
        Serial.println(buffer);
      #endif

      #if debugGPS
        sprintf(buffer, "%2d:%2d:%2d %2d-%2d-20%2d , valido=%d, %f,%f,%.2f,%.3f,hdop:%.2f,fix:%4d x:%f, y:%f",
          dSt.dato.tiempo[2],dSt.dato.tiempo[1],dSt.dato.tiempo[0],dSt.dato.fecha[0],dSt.dato.fecha[1],dSt.dato.fecha[2],
          (dSt.dato.gpsValido)? 1:0, dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad,dSt.dato.hdop,dSt.dato.tiempoDesdeUltimoFix,
          (dSt.dato.latitud-41.685455)*111317.099692, (dSt.dato.longitud+0.887576)*111317.099692); //radio_de_la_tierra * 2pi / 360
        Serial.println(buffer);
      #endif

      #if debugDatosBytes
        for(int i=0; i<sizeof(dSt.byte); i++){
          sprintf(buffer, "%3d ", dSt.byte[i]);
          Serial.print(buffer);
        }
        Serial.print("bytes:");
        Serial.println(sizeof(dSt.byte));
      #endif
      
      #if debugParaSerialPlotter
        sprintf(buffer, "Variable_1:%f,Variable_2:%f,Variable_3:%f,Variable_4:%f,Variable_5:%f,Variable_6:%f,Variable_7:%d,Variable_8:%f,Variable_9:%f,Variable_10:%f,Variable_11:%f",
          dSt.dato.acc[0],dSt.dato.acc[1],dSt.dato.acc[2],dSt.dato.gyro[0],dSt.dato.gyro[1],dSt.dato.gyro[2],
          (dSt.dato.gpsValido)? 1:0, dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad);
        Serial.println(buffer);
      #endif

      #if debuggearValorMedioSensores
        for(int i=0; i<3; i++){
          mediaAcc[i]  += gananciaParaLaMedia*(dSt.dato.acc[i]-mediaAcc[i]);
          mediaGiro[i] += gananciaParaLaMedia*(dSt.dato.gyro[i]-mediaGiro[i]);
          mediaMagn[i] += gananciaParaLaMedia*(dSt.dato.magn[i]-mediaMagn[i]);
        }
        
        sprintf(buffer, "IMU:% 2.9f,% 2.9f,% 2.9f,% 6.6f,% 6.6f,% 6.6f MAG:% 6.6f,% 6.6f,% 6.6f",
          mediaAcc[0],mediaAcc[1],mediaAcc[2],mediaGiro[0],mediaGiro[1],mediaGiro[2],mediaMagn[0],mediaMagn[1],mediaMagn[2]);
        Serial.println(buffer);
      #endif

      cont = 0;
    }
  }

  // Debe llamarse al final de cada ciclo: conserva la impresión original
  // de tiempo de cómputo incluso con modoTest desactivado.
  void mostrarDebug(uint32_t tiempoComputoUs) {
    #if modoTest
      debuggearTodo();
    #endif
    #if debugTiempoComputo
      if (cont == 0) {
        Serial.print("Tiempo computo (us):");
        Serial.println(tiempoComputoUs);
      }
    #endif
  }
////////////////////////////////////
