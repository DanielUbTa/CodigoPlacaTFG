# Receptor Heltec WiFi LoRa 32 V3: CSV por Serial y datos en pantalla

Recibe los paquetes de **64 bytes** del emisor TFG con RadioLib / SX1262 y conserva exactamente la salida **CSV de 16 columnas** del receptor anterior. No utiliza SD ni LittleFS.

## Pantalla OLED integrada (128 × 64)

Muestra siempre el **último valor recibido** de `altura_max` (en metros respecto al despegue), `latitud`, `longitud` y **segundos desde el ultimo paquete LoRa valido**:

```text
ALTURA MAXIMA
320 m
LAT: 41.685100
LON: -0.887800
RX: 0.8s OK
```

Al inicio se muestran ceros y `RX: SIN DATOS`. Tras cada paquete valido, el contador `RX` vuelve a `0.0s`; crece mientras no llegan paquetes. A partir de 5 segundos aparece `SIN SENAL` y se mantienen las ultimas medidas. Si falla la radio se indica `RADIO ERROR` o `RADIO ERR`, segun el caso. El contador se muestra en segundos con un decimal hasta `9999` segundos. Si la pantalla no esta disponible, la salida por Serial sigue funcionando.

El display se actualiza cada **250 ms** para reducir actividad I2C; el CSV se imprime en cuanto se valida el paquete, sin esperar a la pantalla.

## Instalación en Arduino IDE

1. Instala el soporte ESP32/Heltec V3 y selecciona **WiFi LoRa 32(V3)**.
2. Instala las bibliotecas **RadioLib**, **Adafruit GFX Library** y **Adafruit SSD1306** (y sus dependencias) desde el gestor de bibliotecas.
3. Abre `Receptor_Heltec_V3_CSV_OLED_Tiempo.ino` con `Protocolo.h` en la misma carpeta; carga el programa.
4. Conecta el monitor serie a **115200** baudios. Verás una única cabecera CSV y una fila por cada paquete válido. **No hay mensajes de pantalla ni avisos por Serial** mientras `DEBUG_RECEPTOR` esté a `0`.

### Configuración de hardware

- SX1262: NSS=8, SCK=9, MOSI=10, MISO=11, RESET=12, BUSY=13, DIO1=14.
- Pantalla SSD1306 I2C: SDA=17, SCL=18, RESET=21, Vext=36 **LOW para alimentar**, dirección `0x3C`.
- LoRa: 868 MHz, SF9, BW125 kHz, CR 4/5, sync `0x38` y preámbulo 8, sin cambios respecto al receptor anterior.

### CSV por Serial

```csv
acc_x,acc_y,acc_z,pitch,roll,yaw,vel,vel_max,latitud,longitud,altura,altura_max,vel_vert,temperatura,estado,flag
0.0100,-0.0200,1.0100,1.00,-2.00,120.00,12.300,84.500,41.685100,-0.887800,180,320,-8.200,23,2,0
```

Los valores `NaN`/`Inf` se convierten en `0`; `flag=-1` se normaliza a `0`. Las tramas con CRC o tamaño incorrectos se descartan (no se generan filas falsas). La pantalla lee exactamente los mismos datos saneados que el CSV.

## Cambios respecto al receptor anterior

Respecto al receptor OLED anterior se anade el tiempo transcurrido desde el ultimo paquete valido en la linea inferior. Se preservan `Protocolo.h`, la estructura binaria, radio y formateo de CSV.

**Validación:** la sintaxis y el protocolo pueden comprobarse localmente, pero no se ha compilado con las bibliotecas ESP32/Adafruit instaladas en la placa ni probado físicamente.
