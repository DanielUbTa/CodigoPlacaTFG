# CodigoPlacaTFG — Firmware revisado (2026-11-05)

**Estado: versión experimental para validación en banco. NO validada para vuelo.**

Sigue utilizando los cuatro archivos `.ino` originales:

- `CodigoPlacaTFG.ino`: configuración, FreeRTOS y máquina de estados **lógicos**.
- `Sensores.ino`: IMU LSM6DSO por I2C (WHO_AM_I=`0x6C`), magnetómetro, barómetro, GPS e historial RAM de 128 muestras.
- `Telemetria.ino`: LoRa, SD, Flash QSPI y exportación de vuelos.
- `debug.ino`: diagnóstico por puerto serie, normalmente desactivado en vuelo.

Los cuatro deben estar en una carpeta **`CodigoPlacaTFG`**. El receptor LoRa debe actualizarse al nuevo formato de 64 bytes descrito más abajo.

## Qué se ha corregido respecto a la versión auditada

1. **LoRa:** se utiliza un paquete binario de **64 bytes** con cabecera (4), versión (1), tamaño (1), secuencia (2), datos (54) y CRC16 (2). Los datos son los campos del formato Artemisa III consultado previamente; se mantiene el LR1110 por SPI. Incluye **altura actual** y **altura máxima** relativa al despegue, esta última calculada a cada iteración de la máquina de estados (objetivo 100 Hz), no solo al transmitir. Los campos de orientación son estimaciones geométricas, no una AHRS. Se conserva la finalización explícita de transmisiones asíncronas.

2. **IMU:** se mantiene la inicialización directa de LSM6DSO I2C (`0x6A` / ID `0x6C`) que reprodujo los datos del sketch de prueba. `STATUS_REG` verifica que acelerómetro y giro tienen muestras nuevas, evitando contar como nuevas lecturas repetidas. Rango, ODR y filtros siguen en macros de `Sensores.ino`.
3. **Barómetro:** si la presión permanece **exactamente igual** durante más de 2500 ms, se considera posiblemente congelada; la máquina de estados **no** debe usar ese dato para transiciones. Es una heurística conservadora: hay que probar falsos positivos sobre la placa real.
4. **Estados:** se mantienen botón + antirrebote, 4 g × 32 ciclos, apogeo (>100 m, caída 2 m y 10 confirmaciones), principal (<`alturaMain`, al menos 1 s y 10 confirmaciones) y aterrizaje (5 s con `|v| < 1 m/s`). El aterrizaje además exige que la altura no cambie 1 m durante esa ventana.
5. **Preparación:** no se permite entrar en `esperando_despegue` si no hay IMU/barómetro válidos ni **SD funcional o al menos 180 s estimados de espacio Flash**. El parámetro se cambia en el archivo principal (`FLASH_RESERVA_SEGUNDOS`).
6. **Flash:** se conserva el registro binario anterior de 128 B, confirmación al final y no borrado automático. La búsqueda de la siguiente posición libre es ahora binaria; se recorre hacia atrás para localizar el último vuelo. Sigue grabando desde `ascenso` hasta llenarse, incluso después de `aterrizado`.
7. **Exportación:** al arrancar con SD se exporta el último vuelo como **`L0000001.CSV`** (con ID de vuelo; o `LEGACY.CSV` para datos anteriores sin ID). Primero se graba en un archivo `.TMP` y solo se renombra a `.CSV` al acabar correctamente. Si el CSV ya existe se evita crear otro duplicado. Se conservan los registros Flash.
8. **Registros SD y exportados:** se añade cabecera y columnas de tiempo desde el arranque, ID de vuelo, estado y flags. Los datos siguen separados por **tabuladores**, aunque el nombre termine en `.CSV`.
9. **GPS:** se solicitan mensajes **GGA + RMC** mediante `$PCAS03,1,0,0,0,1,0,0,0*02`; de esta forma podrán actualizarse fecha y velocidad. La hora guardada se deja en UTC; no se aplica +1 fijo. Se captura el punto de despegue solo mientras el cohete esté en tierra y exista fix válido; sin fix no se calculan desplazamientos relativos ficticios.
10. **Debug:** los impresos periódicos se desactivan cuando `modoTest=false`; con modo test el resumen sale cada segundo. Siguen existiendo mensajes de arranque/errores en otros módulos para facilitar el diagnóstico local.

## Protocolo LoRa y altura máxima

La carga anterior de 14 bytes se sustituye por una **trama de 64 bytes**. Debes modificar el receptor: **no es compatible con el decodificador anterior**.

- Bytes 0–3: cabecera `A5 5A C3 3C`.
- Byte 4: versión `1`; byte 5: longitud de datos `54`.
- Bytes 6–7: número de secuencia `uint16_t`, little-endian.
- Bytes 8–61: estructura `TelemetriaArtemisa` empaquetada (54 bytes): `acc_x,acc_y,acc_z,pitch,roll,yaw,vel,vel_max,latitud,longitud,altura,altura_max,vel_vert,temperatura,estado,flag`.
- Bytes 62–63: CRC16-CCITT-FALSE, little-endian, calculado sobre los 62 bytes anteriores.

**Campo nuevo prioritario: `altura_max` en bytes 50 y 51**, entero con signo de 16 bits little-endian, expresado en **metros relativos a `alturaDespegue`**, resolución de 1 m, con saturación a 32767 m. Para obtenerlo en el receptor: `int16_t altura_max = (int16_t)((uint16_t)paquete[50] | ((uint16_t)paquete[51] << 8));`. `altura` (actual) ocupa los bytes 48–49, con las mismas unidades.

La máxima se calcula **cada vez que se actualiza la máquina de estados**, usando solo alturas barométricas marcadas válidas. Se conserva tras apogeo y aterrizaje; antes de despegar se transmite 0. La velocidad GPS se convierte de km/h a m/s. `estado` se traduce a los cuatro códigos del formato de referencia (0=tierra, 1=ascenso, 2=descenso, 3=aterrizado). `flag=-1` indica que no se mide continuidad eléctrica porque el despliegue es mecánico; **el byte antiguo de flags de salud no forma parte de esta trama**, aunque sigue guardándose en memoria y en el diagnóstico.

**Compatibilidad radioeléctrica:** copiar el formato de trama no garantiza interoperabilidad por el aire con la placa Artemisa III, que usa un módulo LoRa por UART distinto del LR1110. Ambos extremos deben compartir parámetros radio y protocolo físico.

## Preparación y condiciones de vuelo

El `loop()` intenta ejecutarse a `FRECUENCIA_LOOP_HZ` (100 Hz por defecto). **No se garantiza la frecuencia real:** escrituras Flash/SD, exportaciones en arranque, I2C o debug pueden provocar retrasos. Medir la latencia máxima en banco.

Durante el arranque con SD presente se **exporta el último vuelo antes de iniciar la radio**, lo que puede tardar mucho si hay muchos registros. **No arrancar la aviónica justo antes del lanzamiento sin esperar al fin de la exportación.**

El historial circular de **128 muestras** se mantiene siempre en RAM. Deliberadamente **no se vuelca en bloque al detectar el despegue**, porque una escritura de 128 registros en ese instante bloquearía la tarea de 100 Hz. Solo la muestra actual se empieza a guardar en SD y Flash desde `ascenso`. Si interesa recuperar también el predespegue, hacerlo después mediante una cola de escritura no bloqueante y marcas de tiempo por muestra, sujeto a pruebas de rendimiento.

La SD sigue sincronizándose cada 250 ms y cerrándose al detectar `aterrizado`. **No es una garantía de ausencia de corrupción** ante un corte de alimentación. La Flash no borra ni sobrescribe datos automáticamente; al llenarse hay que exportar y disponer de un procedimiento de borrado validado (todavía no implementado).

Las salidas `DROGUE` y `PARACAIDAS` solo se ponen en LOW al iniciar; **nunca se activan por software**. Los paracaídas mecánicos son independientes y los estados denominados `drogue_desplegado` / `paracaidas_desplegado` son **estimaciones**, no detecciones del mecanismo real.

## Configuración IMU

Editar en `Sensores.ino`:

```cpp
#define IMU_RANGO_ACEL_G        16
#define IMU_RANGO_GIRO_DPS      2000
#define IMU_FRECUENCIA_ACEL_HZ  416
#define IMU_FRECUENCIA_GIRO_HZ  416
#define IMU_FILTRO_ACEL_DIV_ODR 0
#define IMU_FILTRO_GIRO_ACTIVO  0
#define IMU_FILTRO_GIRO_CODIGO  0
```

Si el ODR es menor que el de la tarea principal, algunas iteraciones no tendrán una muestra nueva y se marcarán como inválidas; por eso conviene mantener ODR ≥ 100 Hz si quieres detectar 32 ciclos consecutivos.

## Bibliotecas / requisitos

Core **Adafruit nRF52**, RadioLib, TinyGPSPlus, LPS de Pololu, Adafruit LIS2MDL, Adafruit SPIFlash y **SdFat 2.x**. No se necesita la librería SparkFun/Carlo `LSM6DS3` para la IMU. La SD usa `SD_SPI` sobre `NRF_SPIM1`, y LoRa usa `SPI` con el cableado existente.

La configuración de radio **868 MHz / 20 dBm y ~5 paquetes/s** se ha conservado para no alterar el enlace, pero **requiere revisión reglamentaria** antes de transmitir en España; no se ha validado su adecuación.

## Pruebas efectuadas con esta entrega

- **PASA:** comprobación C++ sintáctica con interfaces simuladas; no es compilación Arduino real.
- **PASA (esta versión):** estructura empaquetada de 54 B y trama de 64 B, CRC y campos de altura con prueba simulada del máximo a frecuencia de lectura.
- **PASA:** simulación de IMU con ID `0x6C`, registros de configuración, muestras crudas del test y detección de ausencia de muestra nueva.
- **PASA:** Flash simulada: dos vuelos, lectura tras registro interrumpido, exportación única, formato heredado.
- **PASA:** simulación de estados: botón, 32 ciclos de despegue, apogeo, principal, sensor congelado y aterrizaje.
- **NO PROBADO:** compilación con las bibliotecas y la placa real, rendimientos de QSPI/SD, comportamiento tras corte eléctrico real, radio LR1110 + receptor y vuelo físico.

## Pendientes que NO se han implementado por seguridad o falta de información

- **Watchdog y recuperación tras reinicio en vuelo:** no se puede reconstruir de forma segura un estado de vuelo solo con el último registro sin diseñar una estrategia de recuperación completa.
- **Medición de batería:** requiere saber divisor resistivo y pin ADC reales.
- **Medición real de la frecuencia efectiva / latencia máxima:** requiere la placa.
- **Detección infalible de barómetro congelado:** el control de igualdad exacta es solo una medida preventiva, no un test definitivo.
- **Paracaídas:** no se implementa ni autoriza disparo de cargas, pues el mecanismo de recuperación es puramente mecánico.
- **Revisión final de espectro radioeléctrico, autorizaciones de lanzamiento y pruebas de recuperación.**

**Antes de un lanzamiento:** probar alimentación, SD, Flash, sensores, comunicación y estados en banco, sin ninguna carga conectada; verificar que el sistema mecánico de recuperación es independiente.
