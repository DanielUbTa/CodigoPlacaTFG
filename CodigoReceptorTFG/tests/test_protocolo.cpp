#include "../Protocolo.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <cmath>

int main() {
  assert(crc16Trama((const uint8_t*)"123456789", 9) == 0x29B1);
  assert(offsetof(Telemetria, altura) == 40);
  assert(offsetof(Telemetria, altura_max) == 42);
  assert(sizeof(Telemetria) == 54);

  Telemetria t = {};
  t.acc_x = 10.0f;
  t.acc_z = 1.013f;
  t.altura = 180;
  t.altura_max = 320;
  t.vel_vert = -8.2f;
  t.estado = 2;
  t.flag = -1;
  t.latitud = std::numeric_limits<float>::quiet_NaN();
  uint8_t trama[TAM_TRAMA] = {};
  memcpy(trama, CABECERA, 4);
  trama[4] = 1;
  trama[5] = sizeof(Telemetria);
  trama[6] = 0x34;
  trama[7] = 0x12;
  memcpy(trama + 8, &t, sizeof(t));
  uint16_t crc = crc16Trama(trama, TAM_TRAMA - 2);
  trama[62] = uint8_t(crc);
  trama[63] = uint8_t(crc >> 8);

  Telemetria resultado;
  uint16_t secuencia = 0;
  assert(decodificarTrama(trama, 64, resultado, secuencia));
  assert(secuencia == 0x1234);
  completarDatos(resultado);
  assert(resultado.altura == 180 && resultado.altura_max == 320);
  assert(resultado.vel_vert == -8.2f && resultado.acc_x == 10.0f);
  assert(resultado.flag == 0 && resultado.latitud == 0.0f);
  assert(!decodificarTrama(trama, 63, resultado, secuencia));
  trama[50] ^= 1;
  assert(!decodificarTrama(trama, 64, resultado, secuencia));
  trama[50] ^= 1;
  trama[4] = 2;
  assert(!decodificarTrama(trama, 64, resultado, secuencia));
  // Comprobar que el CSV generado tiene 16 columnas y que los campos invalidos son cero.
  char linea[256];
  int longitudCSV = formatearLineaCSV(linea, sizeof(linea), resultado);
  assert(longitudCSV > 0 && longitudCSV < (int)sizeof(linea));
  int columnas = 1;
  for (int i = 0; i < longitudCSV; i++) if (linea[i] == ',') columnas++;
  assert(columnas == 16);
  assert(std::strstr(linea, ",180,320,-8.200,") != nullptr);
  assert(std::strstr(linea, ",0.000000,") != nullptr);  // GPS NaN sustituido por 0
  assert(linea[longitudCSV - 1] == '\n');
  assert(std::strstr(linea, ",2,0\n") != nullptr);  // Estado y flag=0
  printf("OK: protocolo, CRC, CSV de 16 columnas y valores no disponibles=0\n");
  return 0;
}
