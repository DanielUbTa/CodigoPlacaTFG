#ifndef PROTOCOLO_ARTEMISA_H
#define PROTOCOLO_ARTEMISA_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// Formato binario del emisor Artemisa III: 54 bytes de datos en una trama de 64.
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

// Campos no disponibles -> 0, tal como se solicita para el CSV.
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
  if (t.estado < 0 || t.estado > 3) t.estado = 0;
  if (t.flag < 0) t.flag = 0; // -1 del emisor significa continuidad NO medida.
}

constexpr const char *CABECERA_CSV =
    "acc_x,acc_y,acc_z,pitch,roll,yaw,vel,vel_max,latitud,longitud,"
    "altura,altura_max,vel_vert,temperatura,estado,flag\n";

// Genera una linea CSV, sin cabecera. Debe llamarse con datos saneados.
inline int formatearLineaCSV(char *destino, size_t capacidad, const Telemetria &t) {
  return snprintf(destino, capacidad,
    "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%.6f,%.6f,%d,%d,%.3f,%d,%d,%d\n",
    t.acc_x, t.acc_y, t.acc_z, t.pitch, t.roll, t.yaw,
    t.vel, t.vel_max, t.latitud, t.longitud,
    (int)t.altura, (int)t.altura_max, t.vel_vert,
    (int)t.temperatura, (int)t.estado, (int)t.flag);
}

#endif
