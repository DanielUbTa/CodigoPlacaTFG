# Receptor LoRa -> CSV por Serial (Heltec WiFi LoRa 32 V3)

Este proyecto **no guarda archivos en la Flash ni en una SD**. Recibe tramas binarias LoRa de 64 bytes y envía al ordenador, por USB Serial, una fila CSV por cada paquete válido.

## Cómo utilizarlo

1. Abre `Receptor_Heltec_V3_CSV_Serial.ino` en Arduino IDE (carpeta con el mismo nombre).
2. Selecciona **WiFi LoRa 32(V3)**, con soporte ESP32-S3, e instala **RadioLib** (API probada mediante interfaces simuladas; se preparó originalmente para RadioLib 7.4.0).
3. Activa el puerto USB serie adecuado de tu configuración de placa. Si usas USB CDC nativo y no aparece el puerto, revisa **USB CDC On Boot**.
4. Carga el sketch y abre el monitor serie a **115200 baudios**. Al arrancar aparecerá una cabecera CSV, seguida de una línea por cada paquete recibido.
5. Puedes usar el registro del monitor serie o un programa en el ordenador para guardar las líneas como `.csv`; el firmware no almacena nada.

## Salida de ejemplo (datos ficticios)

```csv
acc_x,acc_y,acc_z,pitch,roll,yaw,vel,vel_max,latitud,longitud,altura,altura_max,vel_vert,temperatura,estado,flag
0.0100,-0.0200,1.0100,1.00,-2.00,120.00,12.300,84.500,41.685100,-0.887800,180,320,-8.200,23,2,0
```

- Los **16 campos y su orden** corresponden a la estructura `Telemetria` de Artemisa III.
- Los valores no finitos (`NaN`, `Inf`) se sustituyen por **0**. El `flag=-1`, que indica continuidad no medida en nuestra aviónica, también se transforma en **0**.
- No se inventan filas para tramas ausentes o corruptas; se descartan tras verificar cabecera, versión, longitud y CRC.
- **No se imprime** RSSI, secuencia, mensajes `[RX]` ni texto de depuración en modo normal; la salida es CSV limpio.
- `DEBUG_RECEPTOR = 1` permite ver errores de inicialización LoRa, pero añade líneas de diagnóstico que comienzan por `#` y dejarían de ser un CSV estricto.

## Configuración de radio

La configuración por defecto apunta a **nuestra aviónica LR1110**: 868 MHz, 125 kHz, SF9, coding rate 4/5, sync word 0x38 y preámbulo 8. La Heltec V3 incorpora SX1262 y utiliza CS 8, SCK 9, MOSI 10, MISO 11, RST 12, BUSY 13 y DIO1 14.

La **otra placa Artemisa III** utiliza un E220 por UART, canal diferente. Este sketch no recibe simultáneamente ese segundo enlace ni garantiza compatibilidad directa con su modulación. Su estructura de datos es la misma, pero el enlace físico no lo es.

## Archivos

- `Receptor_Heltec_V3_CSV_Serial.ino`: inicialización de SX1262, recepción y escritura CSV por Serial.
- `Protocolo.h`: estructura de 16 campos, CRC16, decodificación, limpieza y formateo CSV.
- `tests/test_protocolo.cpp`: prueba de trama y valores no disponibles, incluido CSV.

Referencia: https://github.com/AkaneCelorrioGonzalvo/Electronica-Artemisa-III/tree/main/Placa%20Junio/Codigo_Receptor

**Validación:** prueba de compilación con g++ para el protocolo, pero no se ha compilado ni ejecutado con Arduino IDE y hardware real en esta sesión.
