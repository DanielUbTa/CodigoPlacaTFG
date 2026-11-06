/*
 * RECEPTOR LORA -> CSV POR SERIAL + PANTALLA OLED
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + SSD1306)
 *
 * Por Serial: cabecera y una fila CSV por cada paquete LoRa valido.
 * En OLED: altura maxima, latitud, longitud y tiempo desde el ultimo paquete valido.
 * Los datos que no esten disponibles se muestran como 0.
 * No guarda archivos y no mezcla mensajes de debug con el CSV.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Protocolo.h"

//////////////////////////////////// Configuracion
  #define BAUDIOS_SERIAL    115200
  #define DEBUG_RECEPTOR    0       // 0 = solo CSV; 1 = mensajes # de diagnostico

  #define FRECUENCIA_MHZ    868.0f
  #define ANCHO_BANDA_KHZ   125.0f
  #define SPREADING_FACTOR  9
  #define CODING_RATE       5       // LoRa 4/5
  #define SYNC_WORD         0x38
  #define PREAMBULO         8

  #define PERIODO_OLED_MS   250
  #define SIN_SENAL_MS      5000
////////////////////////////////////

//////////////////////////////////// Pines Heltec V3
  #define LORA_CS     8
  #define LORA_SCK    9
  #define LORA_MOSI  10
  #define LORA_MISO  11
  #define LORA_RST   12
  #define LORA_BUSY  13
  #define LORA_DIO1  14

  #define OLED_SDA   17
  #define OLED_SCL   18
  #define OLED_RST   21
  #define OLED_VEXT  36    // LOW = alimentacion activada
  #define OLED_ADDR  0x3C
////////////////////////////////////

//////////////////////////////////// Radio y pantalla
  SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
  Adafruit_SSD1306 pantalla(128, 64, &Wire, -1);
  volatile bool paqueteDisponible = false;
  bool radioLista = false;
  bool pantallaLista = false;

  Telemetria ultimosDatos = {};
  bool datosRecibidos = false;
  uint32_t ultimoPaqueteMs = 0;
  uint32_t ultimaPantallaMs = 0;

  void IRAM_ATTR interrupcionLoRa() {
    paqueteDisponible = true;
  }

  void iniciarRadio() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
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

  void iniciarPantalla() {
    pinMode(OLED_VEXT, OUTPUT);
    digitalWrite(OLED_VEXT, LOW);
    delay(100);

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(10);
    digitalWrite(OLED_RST, HIGH);
    delay(10);

    Wire.begin(OLED_SDA, OLED_SCL);
    // Wire ya esta iniciado y hemos reiniciado la pantalla manualmente.
    pantallaLista = pantalla.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
    if (pantallaLista) {
      pantalla.setTextColor(SSD1306_WHITE);
      pantalla.setTextWrap(false);
    }
  }
////////////////////////////////////

//////////////////////////////////// Pantalla OLED
  void actualizarPantalla() {
    if (!pantallaLista || millis() - ultimaPantallaMs < PERIODO_OLED_MS) return;
    ultimaPantallaMs = millis();

    pantalla.clearDisplay();
    pantalla.setTextSize(1);
    pantalla.setCursor(0, 0);
    pantalla.print("ALTURA MAXIMA");

    pantalla.setTextSize(2);
    pantalla.setCursor(0, 12);
    pantalla.print(ultimosDatos.altura_max);
    pantalla.print(" m");

    pantalla.setTextSize(1);
    pantalla.setCursor(0, 37);
    pantalla.print("LAT: ");
    pantalla.print(ultimosDatos.latitud, 6);

    pantalla.setCursor(0, 48);
    pantalla.print("LON: ");
    pantalla.print(ultimosDatos.longitud, 6);

    pantalla.setCursor(0, 56);
    if (!radioLista && !datosRecibidos) pantalla.print("RADIO ERROR");
    else if (!datosRecibidos) pantalla.print("RX: SIN DATOS");
    else {
      uint32_t tiempoSinDatos = millis() - ultimoPaqueteMs;
      pantalla.print("RX: ");
      if (tiempoSinDatos > 9999900UL) pantalla.print(">9999");
      else pantalla.print(tiempoSinDatos / 1000.0f, 1);
      pantalla.print("s ");
      if (!radioLista) pantalla.print("RADIO ERR");
      else if (tiempoSinDatos > SIN_SENAL_MS) pantalla.print("SIN SENAL");
      else pantalla.print("OK");
    }
    pantalla.display();
  }
////////////////////////////////////

//////////////////////////////////// Recepcion y CSV
  void recibirTelemetria() {
    if (!radioLista || !paqueteDisponible) return;
    paqueteDisponible = false;

    uint8_t trama[TAM_TRAMA];
    size_t longitud = radio.getPacketLength();
    int resultado = (longitud == TAM_TRAMA)
      ? radio.readData(trama, TAM_TRAMA) : -1;

    // Rearmar RX antes de dibujar o escribir por Serial.
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
    ultimosDatos = t;            // Mismos datos para el CSV y la pantalla.
    datosRecibidos = true;
    ultimoPaqueteMs = millis();

    char linea[256];
    int n = formatearLineaCSV(linea, sizeof(linea), t);
    if (n > 0 && (size_t)n < sizeof(linea))
      Serial.write((const uint8_t *)linea, (size_t)n);
  }
////////////////////////////////////

void setup() {
  Serial.begin(BAUDIOS_SERIAL);
  iniciarPantalla();
  iniciarRadio();
  Serial.print(CABECERA_CSV);
}

void loop() {
  recibirTelemetria();
  actualizarPantalla();
  delay(1);
}
