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

// Pines de SD originales.
#define SD_nCS  8
#define SD_SCK  13
#define SD_MOSI 14
#define SD_MISO 15

#define INTERVALO_SYNC_SD_MS 250  // Confirmar en SD cada 250 ms (no en cada muestra)
#define VELOCIDAD_SPI_SD_MHZ 8    // Velocidad conservadora para empezar las pruebas


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

  // No sobrescribe nunca. Soporta registros antiguos, anteriores al ID de vuelo.
  bool localizarFinalFlash() {
    uint8_t bloque[TAM_REGISTRO_FLASH] __attribute__((aligned(4)));
    uint8_t estadoAnterior = 0xFF;
    uint32_t tiempoAnterior = 0;
    bool hayRegistroAnterior = false;
    for (uint32_t dir = 0; dir + TAM_REGISTRO_FLASH <= flash.size();
         dir += TAM_REGISTRO_FLASH) {
      if (flash.readBuffer(dir, bloque, sizeof(bloque)) != sizeof(bloque)) return false;
      bool vacio = true;
      for (size_t j = 0; j < sizeof(bloque); j++) {
        if (bloque[j] != 0xFF) { vacio = false; break; }
      }
      if (vacio) { siguienteDireccionFlash = dir; return true; }

      RegistroFlash registro = {};
      memcpy(&registro, bloque, sizeof(registro));
      if (dir == 0 && registro.marca != MARCA_REGISTRO_FLASH) {
        Serial.println("Flash con datos desconocidos; NO se sobrescribe.");
        return false;
      }
      if (!registroFlashValido(registro)) continue; // Corte de alimentación: celda parcial
      if (tieneIDVueloValido(registro)) {
        if (registro.vuelo > ultimoVueloFlash) ultimoVueloFlash = registro.vuelo;
        ultimoVueloConIdentificador = true;
      } else {
        // Compatible con registros antiguos sin ID: detectar último ASCENSO.
        unionDatos muestraAntigua = {};
        memcpy(muestraAntigua.byte, registro.datos, sizeof(muestraAntigua.byte));
        const uint8_t estado = (uint8_t)muestraAntigua.dato.estado;
        if (estado == ascenso &&
            (!hayRegistroAnterior || estadoAnterior != ascenso ||
             registro.tiempo_ms < tiempoAnterior)) inicioUltimoVueloLegado = dir;
        estadoAnterior = estado;
        tiempoAnterior = registro.tiempo_ms;
        hayRegistroAnterior = true;
      }
    }
    siguienteDireccionFlash = flash.size();
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
          flagsSensoresActivos |= SDfuncionando;
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
    int lecturaAcelerometro = (int)((isfinite(dSt.dato.acc[0]) ? dSt.dato.acc[0] : 0.0f) * (65536.0/16.0));
    datosAmandar[0] = lecturaAcelerometro & 0xFF;
    datosAmandar[1] = (lecturaAcelerometro>>8) & 0xFF;

    int xMagn = (int)((isfinite(dSt.dato.magn[0]) ? dSt.dato.magn[0] : 0.0f) * 20);
    int yMagn = (int)((isfinite(dSt.dato.magn[1]) ? dSt.dato.magn[1] : 0.0f) * 20);
    int zMagn = (int)((isfinite(dSt.dato.magn[2]) ? dSt.dato.magn[2] : 0.0f) * 20);
    datosAmandar[2] = xMagn & 0xFF;
    datosAmandar[3] = (xMagn>>8) & 0xF | (yMagn<<4) & 0xF0;
    datosAmandar[4] = (yMagn>>4) & 0xFF;
    datosAmandar[5] = zMagn & 0xFF;

    int altura = (int)(isfinite(dSt.dato.altitud_barometro) ? dSt.dato.altitud_barometro : alturaDespegue) - alturaDespegue;
    datosAmandar[6] = (zMagn>>8) & 0xF | (altura<<4) & 0xF0;
    datosAmandar[7] = (altura>>4) & 0xFF;

    int metrosNorte = isfinite(dSt.dato.latitud) ? (int)((dSt.dato.latitud-latitudDespegue)*40075000/360.0) : 0;
    int metrosEste = (isfinite(dSt.dato.longitud) && isfinite(dSt.dato.latitud)) ? (int)((dSt.dato.longitud-longitudDespegue)*40075000/360.0) * cos(dSt.dato.latitud*2.0*M_PI/360.0) : 0; //Para la distorsión cerca de los polos
    datosAmandar[8] = metrosNorte & 0xFF;
    datosAmandar[9] = (metrosNorte>>8) & 0xF | (metrosEste<<4) & 0xF0;
    datosAmandar[10] = (metrosEste>>4) & 0xFF;

    int temp = (int)((isfinite(dSt.dato.temperatura) ? dSt.dato.temperatura : 20.0f)*10.0-200.0);
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


  // Al encender, si hay SD y registros de vuelo, exportar el ÚLTIMO vuelo.
  // LOG000.CSV... nunca sobrescribe archivos ni borra la Flash.
  bool exportarUltimoVueloASD() {
    if (!sdInicializada || !flashInicializada) return false;
    if (!ultimoVueloConIdentificador && inicioUltimoVueloLegado == UINT32_MAX) {
      Serial.println("[FLASH] Sin vuelos anteriores para exportar.");
      return false;
    }

    // El último vuelo puede ser el nuevo formato (ID) o datos heredados.
    const bool conID = ultimoVueloConIdentificador;
    const uint32_t vuelo = ultimoVueloFlash;
    char nombre[13] = {};
    for (unsigned int i = 0; i < 1000; i++) {
      snprintf(nombre, sizeof(nombre), "LOG%03u.CSV", i);
      if (!SD.exists(nombre)) break;
      nombre[0] = 0;
    }
    if (!nombre[0]) {
      Serial.println("[FLASH] Sin nombres LOG libres.");
      return false;
    }

    File destino = SD.open(nombre, FILE_WRITE);
    if (!destino) {
      Serial.println("[FLASH] Error creando CSV de recuperación.");
      return false;
    }

    unionDatos original = dSt;
    uint32_t recuperados = 0, descartados = 0;
    bool correcto = true;
    RegistroFlash registro;
    for (uint32_t dir = 0; dir < siguienteDireccionFlash; dir += TAM_REGISTRO_FLASH) {
      if (flash.readBuffer(dir, (uint8_t*)&registro, sizeof(registro)) != sizeof(registro)) {
        correcto = false;
        break;
      }
      if (!registroFlashValido(registro)) { descartados++; continue; }
      if (conID) {
        if (!tieneIDVueloValido(registro) || registro.vuelo != vuelo) continue;
      } else if (dir < inicioUltimoVueloLegado || tieneIDVueloValido(registro)) continue;

      memcpy(dSt.byte, registro.datos, sizeof(dSt.byte));
      const int n = datosAcsv(buffer);
      destino.print(registro.tiempo_ms);
      destino.write((uint8_t)'\t');
      if (destino.write((const uint8_t*)buffer, n) != (size_t)n) {
        correcto = false;
        break;
      }
      recuperados++;
    }
    dSt = original;
    if (!destino.sync()) correcto = false;
    destino.close();

    Serial.print("[FLASH] Último vuelo -> "); Serial.print(nombre);
    Serial.print(" muestras="); Serial.print(recuperados);
    Serial.print(" incompletas="); Serial.print(descartados);
    Serial.print(" resultado="); Serial.println(correcto ? "OK" : "ERROR");
    return correcto && recuperados > 0;
  }

  char guardarEnMemorias(bool enFlash, bool enSD){
    char out = 0;// Si no es 0 ha habido un error
    // En Flash solamente, no gastar tiempo formateando cada muestra como CSV.
    int longitud = 0;
    if (enSD) { longitud = datosAcsv(buffer); bufferLength = longitud; }
    if (enFlash && flashListaParaGuardar && !guardarMuestraFlash()) out |= 0x1;
    if(enSD){
      if(archivo_salida && !errorEscrituraSD){
        int longitudSD = archivo_salida.print(buffer);
        if(longitudSD != longitud){
          errorEscrituraSD = true; // No seguir escribiendo tras una línea incompleta.
          out |= 0x2;
          flagsSensoresActivos &= ~SDfuncionando;
        }else{
          datosPendientesSD = true;
        }

        // SdFat ya utiliza una caché. sync() confirma datos y metadatos,
        // pero hacerlo en CADA ciclo provoca paradas innecesarias.
        if(datosPendientesSD && millis() - ultimoSyncSD >= INTERVALO_SYNC_SD_MS){
          ultimoSyncSD = millis();
          if(archivo_salida.sync()){
            datosPendientesSD = false;
            if(!errorEscrituraSD) flagsSensoresActivos |= SDfuncionando;
          }else{
            out |= 0x2;
            flagsSensoresActivos &= ~SDfuncionando;
          }
        }
      }else{
        out |= 0x2;
        flagsSensoresActivos &= ~SDfuncionando;
      }
    }
    //Serial.print(buffer);

    return out;
  }
  // Se llama una vez al aterrizar, para no dejar datos pendientes en la SD.
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
