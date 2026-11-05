# Protocolo de telemetría LoRa — CodigoPlacaTFG

**Versión del protocolo:** 1  
**Archivo que lo implementa:** `Telemetria.ino`, funciones `comprimirParaTransmision()` y `enviarTelemetriaSiToca()`  
**Paquete transmitido:** **64 bytes binarios**; no es una línea de texto ni un CSV.  
**Radio de esta placa:** LR1110 controlado por SPI mediante RadioLib.

> **Importante para el receptor:** este formato **no es compatible** con el antiguo paquete de 14 bytes. El receptor debe esperar exactamente 64 bytes y validar su cabecera, versión, tamaño y CRC.

## 1. Qué se envía

**Solo hay un tipo de mensaje de telemetría.** En cada transmisión se genera una trama con los mismos 16 campos, con los valores más recientes disponibles. No se transmite un mensaje distinto para cada fase de vuelo.

| Campo | Qué representa | Tipo | Unidad | Bytes de la trama |
|---|---|---|---|---|
| `acc_x` | Aceleración eje X | `float32` | g | 8–11 |
| `acc_y` | Aceleración eje Y | `float32` | g | 12–15 |
| `acc_z` | Aceleración eje Z | `float32` | g | 16–19 |
| `pitch` | Inclinación estimada a partir del acelerómetro | `float32` | grados | 20–23 |
| `roll` | Inclinación estimada a partir del acelerómetro | `float32` | grados | 24–27 |
| `yaw` | Ángulo horizontal estimado a partir del magnetómetro | `float32` | grados, 0–360 | 28–31 |
| `vel` | Velocidad horizontal indicada por GPS | `float32` | m/s | 32–35 |
| `vel_max` | Velocidad GPS máxima observada desde el arranque | `float32` | m/s | 36–39 |
| `latitud` | Latitud GPS absoluta | `float32` | grados decimales | 40–43 |
| `longitud` | Longitud GPS absoluta | `float32` | grados decimales | 44–47 |
| `altura` | Altitud barométrica **actual** respecto al despegue | `int16_t` | m | 48–49 |
| **`altura_max`** | **Altitud barométrica máxima** respecto al despegue | `int16_t` | m | **50–51** |
| `vel_vert` | Velocidad vertical calculada del barómetro | `float32` | m/s | 52–55 |
| `temperatura` | Temperatura del barómetro | `int16_t` | °C, sin decimales | 56–57 |
| `estado` | Fase de vuelo codificada para telemetría | `int16_t` | código (tabla siguiente) | 58–59 |
| `flag` | Indicador de continuidad de despliegue | `int16_t` | **−1 = no disponible** | 60–61 |

**Total de datos: 54 bytes.** El resto de la trama son cabecera, control y CRC.

### Aclaraciones de unidades y cálculos

- **Aceleraciones:** se envían en **g**, no en m/s². El valor procede de `dSt.dato.acc[]` (incluidos los offsets que aplique el código de sensores).
- **Orientación:** `pitch` y `roll` se estiman geométricamente desde la aceleración; `yaw` desde `atan2(magn_y, magn_x)`. **No es una orientación AHRS fusionada**, y `yaw` no tiene compensación por inclinación. En vuelo acelerado estos ángulos pueden no representar la actitud real.
- **Velocidades GPS:** el dato interno en km/h se divide entre `3.6` para enviar m/s. `vel_max` conserva el máximo observado **desde el arranque**, no necesariamente solo durante el vuelo.
- **Posición GPS:** latitud y longitud **absolutas** (no desplazamientos respecto al lanzamiento). Se envían con la precisión limitada de `float32`.
- **Altitudes:** `altura = altitud_barometro − alturaDespegue`. `altura_max = max(0, alturaMaxima − alturaDespegue)`. La máxima se actualiza durante el vuelo en el ciclo principal, no únicamente al transmitir, y se conserva después del apogeo.
- **Velocidad vertical:** positiva en ascenso, negativa en descenso, calculada desde el barómetro; no es velocidad GPS.
- **Enteros de altitud y temperatura:** se **truncan** hacia cero y se limitan al intervalo `−32768…32767`. La temperatura pierde sus decimales.
- **Datos no disponibles:** el código transmite **0** cuando no dispone de una lectura válida en varios campos (`vel`, posición, orientación, altitud o `vel_vert`). Por tanto, **un 0 no demuestra por sí solo que la medida sea válida**. Esta trama no lleva flags de validez por sensor.

## 2. Valores del campo `estado`

El firmware tiene **seis estados internos**, pero los agrupa en **cuatro valores transmitidos**:

| Valor enviado | Significado para el receptor | Estado(s) internos |
|---:|---|---|
| `0` | En tierra / esperando | `cohete_desarmado`, `esperando_despegue` |
| `1` | Ascenso | `ascenso` |
| `2` | Descenso / recuperación mecánica | `drogue_desplegado`, `paracaidas_desplegado` |
| `3` | Aterrizado | `aterrizado` |

**El campo `estado` describe una estimación lógica del firmware; no confirma físicamente que un paracaídas mecánico se haya abierto.**

### Campo `flag`

Se envía **siempre `-1`**. Los paracaídas son mecánicos y esta placa no mide continuidad eléctrica de cargas. **No interpretar `-1` como continuidad correcta.**

Los antiguos bits de salud (`IMUfuncionando`, `BARfuncionando`, `SDfuncionando`, etc.) **no van dentro del paquete de 64 bytes**; siguen existiendo para el diagnóstico y los registros locales.

## 3. Estructura exacta de los 64 bytes

| Offset (desde 0) | Tamaño | Contenido | Valor / codificación |
|---|---:|---|---|
| 0–3 | 4 B | Cabecera | `A5 5A C3 3C` (hexadecimal) |
| 4 | 1 B | Versión | `01` |
| 5 | 1 B | Tamaño de datos | `36` hex = **54** decimal |
| 6–7 | 2 B | Secuencia | `uint16_t`, little-endian |
| 8–61 | 54 B | Datos | Los 16 campos de la tabla anterior, en ese orden |
| 62–63 | 2 B | CRC | `CRC16-CCITT-FALSE`, little-endian |

La secuencia empieza en 0 tras arrancar, aumenta cada vez que se construye un paquete y vuelve a 0 después de 65535. **No es una marca temporal ni garantiza que el paquete haya llegado al receptor.**

**Orden de bytes:** todos los `float32` siguen IEEE-754 de 32 bits y todos los `int16_t` tienen signo; en esta placa nRF52 se copian en **little-endian** (byte menos significativo primero).

### CRC

- Algoritmo: **CRC-16/CCITT-FALSE**.
- Polinomio: `0x1021`.
- Valor inicial: `0xFFFF`.
- Sin reflexión de entrada ni salida; XOR final `0x0000`.
- Datos cubiertos: **bytes 0–61**, inclusive (62 bytes).
- Resultado transmitido: byte bajo en posición **62** y byte alto en **63**.

El receptor debe **rechazar** una trama si el tamaño, cabecera, versión, longitud interna o CRC no coinciden.

## 4. Ejemplo interpretado (valores ficticios)

La siguiente tabla **no es una captura real de radio**. Muestra cómo leer una transmisión durante el descenso:

| Campo | Ejemplo | Interpretación |
|---|---:|---|
| `acc_x, acc_y, acc_z` | `0.10, −0.15, 0.95` | Aceleración en g |
| `pitch, roll, yaw` | `−5.8, −9.0, 125.4` | Ángulos geométricos en grados |
| `vel, vel_max` | `12.5, 83.2` | Velocidad GPS actual y máxima, m/s |
| `latitud, longitud` | `41.6851, −0.8878` | Coordenadas GPS absolutas |
| `altura, altura_max` | `180, 320` | A 180 m del despegue; máximo alcanzado 320 m |
| `vel_vert` | `−8.2` | Descenso a 8.2 m/s |
| `temperatura` | `24` | 24 °C, sin decimales |
| `estado` | `2` | Descenso |
| `flag` | `−1` | Continuidad no medida |

**Especialmente importante:** cuando el cohete desciende, `altura` baja, pero `altura_max` conserva el máximo registrado.

## 5. Decodificar el mensaje en el receptor

Ejemplo en **Python 3**, sin bibliotecas adicionales. Recibe exactamente los 64 bytes del paquete LoRa:

```python
import struct

CABECERA = bytes.fromhex('A5 5A C3 3C')
CAMPOS = (
    'acc_x', 'acc_y', 'acc_z', 'pitch', 'roll', 'yaw',
    'vel', 'vel_max', 'latitud', 'longitud',
    'altura', 'altura_max', 'vel_vert',
    'temperatura', 'estado', 'flag'
)


def crc16_ccitt_false(datos):
    crc = 0xFFFF
    for byte in datos:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def decodificar_trama(paquete: bytes):
    if len(paquete) != 64:
        raise ValueError('Longitud incorrecta')
    if paquete[:4] != CABECERA or paquete[4] != 1 or paquete[5] != 54:
        raise ValueError('Cabecera, versión o longitud incorrectas')

    crc_recibido = int.from_bytes(paquete[62:64], 'little')
    if crc_recibido != crc16_ccitt_false(paquete[:62]):
        raise ValueError('CRC incorrecto')

    secuencia = int.from_bytes(paquete[6:8], 'little')
    valores = struct.unpack('<10fhhfhhh', paquete[8:62])
    return {'secuencia': secuencia, **dict(zip(CAMPOS, valores))}
```

También puedes recuperar únicamente la **altura máxima** (una vez comprobada la integridad de la trama):

```cpp
int16_t altura_max = (int16_t)((uint16_t)paquete[50] |
                                ((uint16_t)paquete[51] << 8));
```

## 6. Envío y compatibilidad

- **Frecuencia objetivo:** aproximadamente **5 paquetes por segundo** con `FRECUENCIA_LOOP_HZ = 100`; depende de `ciclosAsaltarTransmision`, de que la radio esté libre y de los retrasos del programa. No es una garantía temporal.
- **Transmisión:** LoRa LR1110 a través de RadioLib; el envío es asíncrono y se completa mediante `finishTransmit()`.
- **No contiene:** datos individuales de giroscopio, magnetómetro, presión, HDOP, fecha/hora GPS, voltaje de batería, flags de salud ni historial de 128 muestras. Estos datos pueden existir en los registros locales, pero **no están incluidos en esta trama**.
- **Compatibilidad Artemisa III:** se ha adaptado su **estructura lógica de telemetría**, pero la otra placa usa un módulo LoRa E220 por UART. Compartir formato de datos **no garantiza interoperabilidad radioeléctrica**: ambos extremos necesitan una configuración de radio compatible.

**Fuente de verdad:** `Telemetria.ino`. Si se añade, elimina o cambia algún campo de `TelemetriaArtemisa`, debe actualizarse este documento **y el programa receptor** simultáneamente.
