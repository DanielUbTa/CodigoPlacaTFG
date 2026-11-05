/*
 * RECEPTOR LORA -> CSV POR SERIAL
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 *
 * Muestra una cabecera CSV al arrancar y una fila por trama LoRa valida.
 * No utiliza LittleFS ni tarjeta SD. Los campos sin valor se escriben como 0.
 *
 * Fuente de la estructura de datos: Electronica-Artemisa-III / Codigo_Receptor.
 * Protocolo de radio: CodigoPlacaTFG, trama de 64 bytes con CRC16.
 */
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "Protocolo.h"

//////////////////////////////////// Configuracion
  #define BAUDIOS_SERIAL   115200
  #define DEBUG_RECEPTOR   0      // 0 = solo CSV; 1 = permite mensajes de error

  #define FRECUENCIA_MHZ   868.0f
  #define ANCHO_BANDA_KHZ  125.0f
  #define SPREADING_FACTOR 9
  #define CODING_RATE      5     // LoRa 4/5
  #define SYNC_WORD        0x38
  #define PREAMBULO        8
////////////////////////////////////

//////////////////////////////////// Pines Heltec V3
  #define LORA_CS    8
  #define LORA_SCK   9
  #define LORA_MOSI 10
  #define LORA_MISO 11
  #define LORA_RST  12
  #define LORA_BUSY 13
  #define LORA_DIO1 14
////////////////////////////////////

//////////////////////////////////// Radio
  SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
  volatile bool paqueteDisponible = false;
  bool radioLista = false;

  void IRAM_ATTR interrupcionLoRa() {
    paqueteDisponible = true;
  }

  void iniciarRadio() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    // TCXO de la Heltec V3: DIO3 a 1.8 V.
    int estado = radio.begin(FRECUENCIA_MHZ, ANCHO_BANDA_KHZ,
      SPREADING_FACTOR, CODING_RATE, SYNC_WORD, 10, PREAMBULO, 1.8f);
    if (estado == RADIOLIB_ERR_NONE) {
      radio.setPacketReceivedAction(interrupcionLoRa);
      estado = radio.startReceive();
    }
    radioLista = (estado == RADIOLIB_ERR_NONE);

    #if DEBUG_RECEPTOR
      if (!radioLista) {
        Serial.print("# ERROR iniciando LoRa: ");
        Serial.println(estado);
      }
    #endif
  }
////////////////////////////////////

//////////////////////////////////// Recepcion y salida CSV
  void recibirTelemetria() {
    if (!radioLista || !paqueteDisponible) return;
    paqueteDisponible = false;

    uint8_t trama[TAM_TRAMA];
    size_t longitud = radio.getPacketLength();
    int resultado = (longitud == TAM_TRAMA)
      ? radio.readData(trama, TAM_TRAMA) : -1;

    // Reanudar RX antes de convertir e imprimir el paquete.
    int reinicio = radio.startReceive();
    radioLista = (reinicio == RADIOLIB_ERR_NONE);

    #if DEBUG_RECEPTOR
      if (!radioLista) {
        Serial.print("# ERROR reiniciando RX: ");
        Serial.println(reinicio);
      }
    #endif
    if (resultado != RADIOLIB_ERR_NONE) return;

    Telemetria t;
    uint16_t secuencia;
    if (!decodificarTrama(trama, TAM_TRAMA, t, secuencia)) return;

    completarDatos(t);
    char linea[256];
    int n = formatearLineaCSV(linea, sizeof(linea), t);
    if (n > 0 && (size_t)n < sizeof(linea))
      Serial.write((const uint8_t *)linea, (size_t)n);
  }
////////////////////////////////////

void setup() {
  Serial.begin(BAUDIOS_SERIAL);
  iniciarRadio();
  Serial.print(CABECERA_CSV);
}

void loop() {
  recibirTelemetria();
  delay(1);
}
