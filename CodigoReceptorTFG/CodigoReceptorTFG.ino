/*
 * RECEPTOR LORA -> JSON POR SERIAL + PANTALLA OLED
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + SSD1306)
 *
 * Por Serial: una linea JSON por cada paquete LoRa valido.
 * En OLED: altura maxima, latitud, longitud, estado, ultimo RX, GPS y SNR.
 * Los datos que no esten disponibles se muestran como 0.
 * No guarda archivos ni mezcla debug con el JSON.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Estructura de telemetria declarada antes de los prototipos automaticos.
struct Telemetria;

#ifndef PROTOCOLO_ARTEMISA_H
#define PROTOCOLO_ARTEMISA_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// Trama de 64 bytes compatible en estructura con Artemisa III.
// El campo estado usa los seis estados logicos del emisor TFG (0..5).
struct __attribute__((packed)) Telemetria {
  float acc_x, acc_y, acc_z;
  float pitch, roll, yaw;
  float vel, vel_max;
  float latitud, longitud;
  int16_t altura, altura_max;
  float vel_vert;
  int16_t temperatura, estado, flag;
};

static_assert(sizeof(float) == 4 && sizeof(Telemetria) == 54,
              "El formato de telemetria necesita float de 32 bits y 54 bytes");
static_assert(sizeof(int16_t) == 2, "Se requiere int16_t de 2 bytes");

constexpr size_t TAM_TRAMA = 64;
constexpr uint8_t CABECERA[4] = {0xA5, 0x5A, 0xC3, 0x3C};

inline uint16_t crc16Trama(const uint8_t *datos, size_t longitud) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < longitud; ++i) {
    crc ^= (uint16_t)datos[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                             : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline bool decodificarTrama(const uint8_t *datos, size_t longitud,
                            Telemetria &telemetria, uint16_t &secuencia) {
  if (longitud != TAM_TRAMA || memcmp(datos, CABECERA, 4) != 0 ||
      datos[4] != 1 || datos[5] != sizeof(Telemetria)) return false;

  const uint16_t crcRecibido = (uint16_t)datos[62] | ((uint16_t)datos[63] << 8);
  if (crcRecibido != crc16Trama(datos, 62)) return false;

  secuencia = (uint16_t)datos[6] | ((uint16_t)datos[7] << 8);
  memcpy(&telemetria, datos + 8, sizeof(telemetria));
  return true;
}

inline float numeroValido(float valor) {
  return isfinite(valor) ? valor : 0.0f;
}

// Campos no disponibles -> 0, tal como se solicita para el JSON.
inline void completarDatos(Telemetria &t) {
  t.acc_x = numeroValido(t.acc_x);
  t.acc_y = numeroValido(t.acc_y);
  t.acc_z = numeroValido(t.acc_z);
  t.pitch = numeroValido(t.pitch);
  t.roll = numeroValido(t.roll);
  t.yaw = numeroValido(t.yaw);
  t.vel = numeroValido(t.vel);
  t.vel_max = numeroValido(t.vel_max);
  t.latitud = numeroValido(t.latitud);
  t.longitud = numeroValido(t.longitud);
  t.vel_vert = numeroValido(t.vel_vert);
  if (t.estado < 0 || t.estado > 5) t.estado = 0;
  if (t.flag < 0) t.flag = 0; // -1 del emisor significa continuidad NO medida.
}

// JSON por lineas, compatible con los 16 nombres y precisiones
// del Codigo_Receptor de Artemisa III (sin "secuencia").
// No requiere ArduinoJson ni reserva dinamica con String.
inline int formatearLineaJSON(char *destino, size_t capacidad, const Telemetria &t) {
  return snprintf(destino, capacidad,
    "{\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,"
    "\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,"
    "\"vel\":%.2f,\"vel_max\":%.2f,"
    "\"latitud\":%.6f,\"longitud\":%.6f,"
    "\"altura\":%d,\"altura_max\":%d,"
    "\"vel_vert\":%.2f,\"temperatura\":%d,"
    "\"estado\":%d,\"flag\":%d}\n",
    t.acc_x, t.acc_y, t.acc_z, t.pitch, t.roll, t.yaw,
    t.vel, t.vel_max, t.latitud, t.longitud,
    (int)t.altura, (int)t.altura_max, t.vel_vert,
    (int)t.temperatura, (int)t.estado, (int)t.flag);
}
#endif


//////////////////////////////////// Configuracion
  #define BAUDIOS_SERIAL    115200
  #define DEBUG_RECEPTOR    0       // Debe estar en 0 para salida JSON limpia

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
  float ultimoSNR = 0.0f;
  bool snrDisponible = false;

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

//////////////////////////////////// Estado mostrado en la OLED
  const char* nombreEstado(int16_t estado) {
    switch (estado) {
      case 0: return "DESARMADO";
      case 1: return "ARMADO";       // Esperando despegue
      case 2: return "ASCENSO";
      case 3: return "DROGUE";       // Estado logico, no confirma despliegue
      case 4: return "PRINCIPAL";    // Estado logico, no confirma despliegue
      case 5: return "ATERRIZADO";
      default: return "DESCONOCIDO";
    }
  }
////////////////////////////////////

//////////////////////////////////// Pantalla OLED
  // El protocolo no transmite el estado GPS: solo podemos comprobar
  // si han llegado coordenadas distintas de cero y dentro de rango.
  bool posicionGPSDisponible() {
    return datosRecibidos &&
           isfinite(ultimosDatos.latitud) && isfinite(ultimosDatos.longitud) &&
           fabsf(ultimosDatos.latitud) <= 90.0f &&
           fabsf(ultimosDatos.longitud) <= 180.0f &&
           (ultimosDatos.latitud != 0.0f || ultimosDatos.longitud != 0.0f);
  }

  // Cinco barras orientativas. El valor en dB es el dato real.
  uint8_t barrasSNR(float snr) {
    if (snr < -15.0f) return 0;
    if (snr < -10.0f) return 1;
    if (snr <  -5.0f) return 2;
    if (snr <   0.0f) return 3;
    if (snr <   5.0f) return 4;
    return 5;
  }

  void actualizarPantalla() {
    if (!pantallaLista || millis() - ultimaPantallaMs < PERIODO_OLED_MS) return;
    ultimaPantallaMs = millis();

    const uint32_t tiempoSinDatos = millis() - ultimoPaqueteMs;
    const bool enlaceReciente = datosRecibidos && radioLista &&
                                tiempoSinDatos <= SIN_SENAL_MS;
    const bool gpsOK = enlaceReciente && posicionGPSDisponible();

    pantalla.clearDisplay();
    pantalla.setTextSize(1);
    pantalla.setCursor(0, 0);
    pantalla.print("ALTURA MAXIMA");

    pantalla.setTextSize(2);
    pantalla.setCursor(0, 10);
    pantalla.print(ultimosDatos.altura_max);
    pantalla.print(" m");

    pantalla.setTextSize(1);
    pantalla.setCursor(0, 29);
    pantalla.print("LAT:");
    pantalla.print(ultimosDatos.latitud, 6);

    pantalla.setCursor(0, 38);
    pantalla.print("LON:");
    pantalla.print(ultimosDatos.longitud, 6);

    pantalla.setCursor(0, 47);
    pantalla.print("EST:");
    pantalla.print(datosRecibidos ? nombreEstado(ultimosDatos.estado) : "SIN DATOS");

    pantalla.setCursor(0, 56);
    if (!radioLista && !datosRecibidos) pantalla.print("RADIO ERROR");
    else if (!datosRecibidos) pantalla.print("RX:SIN DATOS");
    else {
      pantalla.print("RX:");
      if (tiempoSinDatos > 9999900UL) pantalla.print(">9999");
      else pantalla.print(tiempoSinDatos / 1000.0f, 1);
      pantalla.print("s ");
      if (!radioLista) pantalla.print("ERR");
      else if (tiempoSinDatos > SIN_SENAL_MS) pantalla.print("SIN");
      else pantalla.print("OK");
    }

    // Panel derecho de 32 pixeles: GPS y barras SNR.
    pantalla.drawFastVLine(95, 0, 64, SSD1306_WHITE);
    pantalla.setCursor(101, 1);
    pantalla.print("GPS");
    if (gpsOK) pantalla.fillCircle(105, 17, 3, SSD1306_WHITE);
    else pantalla.drawCircle(105, 17, 3, SSD1306_WHITE);
    pantalla.setCursor(112, 13);
    pantalla.print(gpsOK ? "OK" : "--");

    pantalla.setCursor(101, 28);
    pantalla.print("SNR");
    pantalla.setCursor(101, 39);
    if (enlaceReciente && snrDisponible) {
      pantalla.print((int)roundf(ultimoSNR));  // dB
    } else {
      pantalla.print("--");
    }

    uint8_t barras = (enlaceReciente && snrDisponible)
      ? barrasSNR(ultimoSNR) : 0;
    for (uint8_t i = 0; i < 5; i++) {
      const int x = 100 + i * 5;
      const int alto = 3 + i * 2;
      pantalla.drawRect(x, 63 - alto, 3, alto, SSD1306_WHITE);
      if (i < barras && alto > 2)
        pantalla.fillRect(x + 1, 64 - alto, 1, alto - 2, SSD1306_WHITE);
    }

    pantalla.display();
  }
////////////////////////////////////

//////////////////////////////////// Recepcion y JSON
  void recibirTelemetria() {
    if (!radioLista || !paqueteDisponible) return;
    paqueteDisponible = false;

    uint8_t trama[TAM_TRAMA];
    size_t longitud = radio.getPacketLength();
    int resultado = (longitud == TAM_TRAMA)
      ? radio.readData(trama, TAM_TRAMA) : -1;

    // El SNR corresponde al paquete recibido; leer ANTES de rearmar RX.
    const float snrPaquete = (resultado == RADIOLIB_ERR_NONE)
      ? radio.getSNR() : 0.0f;

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
    ultimosDatos = t;            // Mismos datos para el JSON y la pantalla.
    datosRecibidos = true;
    ultimoPaqueteMs = millis();
    snrDisponible = isfinite(snrPaquete);
    if (snrDisponible) ultimoSNR = snrPaquete;

    char linea[320];
    int n = formatearLineaJSON(linea, sizeof(linea), t);
    if (n > 0 && (size_t)n < sizeof(linea))
      Serial.write((const uint8_t *)linea, (size_t)n);
  }
////////////////////////////////////

void setup() {
  Serial.begin(BAUDIOS_SERIAL);
  iniciarPantalla();
  iniciarRadio();
}

void loop() {
  recibirTelemetria();
  actualizarPantalla();
  delay(1);
}
