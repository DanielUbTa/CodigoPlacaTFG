# CodigoPlacaTFG — Aviónica nRF52

Código de aviónica organizado para facilitar su mantenimiento por los miembros del grupo.

## Archivos

- `CodigoPlacaTFG.ino`: configuración, tarea periódica a 100 Hz y máquina de estados.
- `Sensores.ino`: GPS, IMU, magnetómetro, barómetro y últimos 128 registros en RAM.
- `Telemetria.ino`: radio LoRa, SD y registro binario Flash QSPI.
- `debug.ino`: mensajes y herramientas de depuración.

Colocar **todos los archivos `.ino` en una carpeta llamada `CodigoPlacaTFG`**.

## Bibliotecas

Se utiliza el **core Adafruit nRF52** y las bibliotecas RadioLib, TinyGPSPlus, Pololu LPS,
Adafruit LIS2MDL, Adafruit SPIFlash y SdFat. **No necesita ninguna biblioteca LSM6DS3.**
La IMU detectada (`WHO_AM_I=0x6C`) se controla por I2C con registros de LSM6DSO/LSM6DSOX,
como en el programa de test que devolvió muestras válidas.

## Máquina de estados (lógica; SIN activación eléctrica)

1. `cohete_desarmado`: botón pulsado de forma estable al menos 30 ms y sensores IMU/barómetro disponibles → `esperando_despegue`.
2. `esperando_despegue`: se actualiza la referencia de altura mientras no se detecta aceleración; aceleración de módulo superior a 4 g **32 ciclos consecutivos** → `ascenso`. La altura de referencia se congela al empezar la secuencia de aceleración.
3. `ascenso`: primero se supera **100 m** sobre la referencia; después, descenso de al menos **2 m** desde la máxima altura y velocidad barométrica **≤ 0 m/s** durante **10 ciclos consecutivos** → `drogue_desplegado`.
4. `drogue_desplegado`: tras un mínimo de **1 s** en este estado, velocidad vertical **< −0,5 m/s** y altura relativa **< `alturaMain` (150 m)** durante **10 ciclos** → `paracaidas_desplegado`.
5. `paracaidas_desplegado`: velocidad barométrica **|v| < 1 m/s** durante **5 s continuados** → `aterrizado`.

Las transiciones representan **eventos observados**, no órdenes de despliegue. En este cohete los paracaídas se despliegan **mecánicamente**. Las salidas `DROGUE` y `PARACAIDAS` permanecen a LOW.

Si el apogeo está por debajo de `alturaMain`, el estado de principal puede alcanzarse una vez confirmada la bajada y transcurrido el tiempo mínimo: esto **no significa que el paracaídas se haya abierto físicamente**.

## Almacenamiento

El historial RAM de 128 muestras se actualiza siempre. SD y Flash empiezan a registrar al entrar en `ascenso`.
La **Flash continúa guardando en todos los ciclos (también tras el apogeo y aterrizaje)** hasta llenarse o apagar la placa. La SD se sincroniza cada 250 ms y se cierra al aterrizar.

Al arrancar, **si se detecta una SD**, antes de iniciar la radio se exporta automáticamente el **último vuelo** de Flash en `LOG000.CSV`, `LOG001.CSV`, etc. No se borra la Flash ni se sobrescriben archivos. Si no hay vuelo anterior, no se crea archivo de recuperación vacío. Los archivos `VUELO000.CSV` son los registros nuevos en SD.

Cada registro nuevo de Flash tiene un identificador de vuelo y su complemento, almacenados en bytes libres del bloque de 128 bytes: mantiene la lectura de registros antiguos. En memorias previas sin ID, se deduce el último despegue a partir de los estados guardados. Un corte de alimentación puede dejar una muestra incompleta, que se descarta en la exportación. No se puede garantizar cero corrupción si hay caída de tensión durante escritura FAT. La capacidad total de una W25Q128 es 16 MiB, ~21 min 50 s a 100 Hz / 128 B por muestra **solo si la placa consigue mantener esa cadencia**.

**Atención:** la exportación de miles de registros puede hacer que el arranque tarde considerablemente. La placa no queda lista para una nueva operación hasta que finaliza. La Flash NO se borra automáticamente: cuando se llena dejará de registrar; no hay aún rutina de borrado.

## Advertencias y pruebas pendientes

- Validar las librerías y la compilación con el modelo exacto de placa y las versiones instaladas.
- Comprobar la polaridad del botón y la resistencia externa: `INPUT` supone que la entrada tiene un nivel definido.
- Medir frecuencia real y bloqueos por SD/Flash. `vTaskDelayUntil()` no garantiza 100 Hz si hay escrituras lentas.
- Reproducir vuelos simulados y comprobar las transiciones con sensores desconectados o muestras inválidas.
- Los datos de barómetro pueden ser erróneos o quedarse congelados aun cuando I2C responda. Los estados no son evidencia infalible de despliegue.
- El software **no controla** el despliegue mecánico. La recuperación requiere pruebas mecánicas y una revisión de seguridad independiente.
- Revisar la configuración LoRa y los requisitos reglamentarios de la banda utilizada.

## IMU LSM6DSO (ID 0x6C)

En `Sensores.ino`, al principio, puedes modificar las siguientes constantes:

- `IMU_RANGO_ACEL_G`: 2, 4, 8, 16 g. Se recomienda **16 g** para la condición de despegue >4 g.
- `IMU_RANGO_GIRO_DPS`: 125, 250, 500, 1000 o 2000 °/s.
- `IMU_FRECUENCIA_ACEL_HZ`, `IMU_FRECUENCIA_GIRO_HZ`: 13, 26, 52, 104, 208, 416, 833 o 1660 Hz.
- `IMU_FILTRO_ACEL_DIV_ODR`: 0 (sin LPF2) o divisor de la ODR: 4, 10, 20, 45, 100, 200, 400, 800.
- `IMU_FILTRO_GIRO_ACTIVO`: 0/1; `IMU_FILTRO_GIRO_CODIGO`: 0..7. La frecuencia efectiva de ese filtro depende de la ODR (consultar hoja de datos ST).
- `IMU_REINTENTO_MS`: reintento cuando está desconectada, únicamente antes de `ascenso`.

La IMU usa dirección 0x6A, `WHO_AM_I=0x6C`, BDU y autoincremento. Lee 12 bytes desde el registro 0x22 y convierte a **g y °/s** con sensibilidades del fabricante, sumando los offsets originales. Las lecturas fallidas se marcan con `NaN` para no confirmar falsas transiciones.

## Pruebas imprescindibles

1. Cargar con salidas de recuperación no conectadas y comprobar el diagnóstico `[IMU] ID=0x6C`, seguido de `IMU:OK`.
2. Repetir la prueba en reposo: `|a|≈1 g`, gyro cercano a 0 °/s; mover la placa para ver cambios.
3. Dejar la Flash registrar, reiniciar con SD insertada y verificar que genera `LOGnnn.CSV` del vuelo anterior.
4. Simular en banco los estados hasta `aterrizado`, mantenerla alimentada y comprobar que Flash sigue creciendo hasta llenarse.
5. Interrumpir la alimentación mientras escribe, reiniciar, exportar y comprobar cuántos registros están completos. Nunca hacerlo con sistemas de recuperación activos.

Las salidas DROGUE y PARACAIDAS permanecen desactivadas. La electrónica no ordena el despliegue mecánico.
