/*
  AVIONICA - PROGRAMA PRINCIPAL (nRF52 / Adafruit Arduino Core)

  Solo coordina la placa. Las funciones de sensores y comunicaciones
  se implementan en las otras tres pestañas .ino.

  IMPORTANTE: las transiciones son lógicas, no accionan paracaídas.
*/
struct ConfiguracionLoRa; // Adelantos necesarios para los prototipos de Arduino.
struct RegistroFlash;
union unionDatos;  // Adelanto para el prototipo automatico de datosAcsv().

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <FreeRTOS.h>
#include <task.h>
#include <RadioLib.h>       // Constante RADIOLIB_ERR_NONE

//////////////////////////////////// Ajustes que hay que revisar antes de cada lanzamiento
  #define FRECUENCIA_LOOP_HZ 100
            // Frecuencia objetivo (compilación); antes: 20 ms

  #define modoTest false
            // Ojo que para los lanzamientos esto esté en false
            // true: mostrar depuración completa; false: solo diagnóstico de salud

  // Si hay SD al arrancar, se exporta automáticamente el último vuelo de Flash.

  #define milisegundosAntesDeApogeo 1000
            // Esto será para poder despegar el paracaídas antes o después del apogeo
            // TODAVÍA NO ESTÁ IMPLEMENTADO

  #define alturaMain 150
            // m sobre altura de despegue

  #define CICLOS_CONFIRMAR_APOGEO 10
  #define CICLOS_CONFIRMAR_MAIN 10
  #define TIEMPO_MINIMO_DROGUE_MS 1000
  #define TIEMPO_ESTABILIDAD_ATERRIZAJE_MS 5000
            // Confirmaciones para evitar cambios de estado por ruido

  #define FLASH_RESERVA_SEGUNDOS 180
            // Aviso y bloqueo de armado sin SD si quedan menos de 3 min en Flash

  #define periodoMuestreo (1000.0f / FRECUENCIA_LOOP_HZ)
            // ms por ciclo; utilizado en la detección de apogeo
////////////////////////////////////



//////////////////////////////////// Configuraciones de LoRa (no cambiar es automático)
  #define ciclosAsaltarTransmision ((FRECUENCIA_LOOP_HZ + 4) / 5)

  #if FRECUENCIA_LOOP_HZ < 1 || FRECUENCIA_LOOP_HZ > configTICK_RATE_HZ
    #error "FRECUENCIA_LOOP_HZ fuera de rango para el tick de FreeRTOS"
  #endif
////////////////////////////////////



//////////////////////////////////// Definición de los pines
  #define pin_boton  (32 + 8)
  #define pin_buzzer 3
  #define pin_led    6
  #define DROGUE     7
  #define PARACAIDAS 16
////////////////////////////////////



//////////////////////////////////// Globales generales

  // Referencias utilizadas para enviar telemetría relativa al despegue.
  float alturaDespegue = 220.0f;
  float latitudDespegue = 41.685109f;    // Mantener valores originales por ahora
  float longitudDespegue = -0.887776f;  // Fallback; sin GPS no se envía posición relativa
  bool referenciaGPSValida = false;

////////////////////////////////////




//////////////////////////////////// Máquina de estados
  enum estados {
    cohete_desarmado,
    esperando_despegue,
    ascenso,
    drogue_desplegado,
    paracaidas_desplegado,
    aterrizado
  };

  char estadoDelCohete = cohete_desarmado; // Armado lógico; espera pulsación del botón

  float velocidadVertical = 0.0f;
  bool velocidadValida = false;
  float alturaMaxima = -100000.0f; // Altitud barométrica absoluta máxima de la sesión en vuelo.

  void actualizarMaquinaDeEstados() {
    static uint8_t ciclosAceleracion = 0, ciclosApogeo = 0, ciclosMain = 0;
    static bool botonLecturaAnterior = false, botonEstable = false;
    static uint32_t instanteCambioBoton = 0, instanteApogeo = 0, inicioEstabilidad = 0;
    static float alturaInicioEstabilidad = 0.0f;
    static bool apogeoHabilitado = false;
    static float alturasVelocidad[32] = {};
    static uint32_t tiemposAltura[32] = {};
    static uint8_t indiceAltura = 0, numeroAlturas = 0;

    const uint32_t ahora = millis();
    const bool boton = digitalRead(pin_boton) == LOW;
    const bool barOK = alturaActualValida();
    const float altura = obtenerAlturaBarometricaActual();

    // Antirrebote del botón: registrar solo una nueva pulsación estable.
    if (boton != botonLecturaAnterior) instanteCambioBoton = ahora;
    bool nuevaPulsacion = false;
    if (ahora - instanteCambioBoton >= 30 && boton != botonEstable) {
      botonEstable = boton;
      nuevaPulsacion = boton;
    }
    botonLecturaAnterior = boton;

    // Velocidad vertical: diferencia barométrica entre muestras separadas ~320 ms.
    velocidadValida = false;
    if (barOK) {
      if (numeroAlturas == 32) {
        uint32_t dt = ahora - tiemposAltura[indiceAltura];
        if (dt > 0) {
          velocidadVertical = (altura - alturasVelocidad[indiceAltura]) * 1000.0f / dt;
          velocidadValida = isfinite(velocidadVertical);
        }
      }
      alturasVelocidad[indiceAltura] = altura;
      tiemposAltura[indiceAltura] = ahora;
      indiceAltura = (indiceAltura + 1) & 31;
      if (numeroAlturas < 32) numeroAlturas++;
    } else {
      numeroAlturas = 0;
      indiceAltura = 0;
    }

    // Registrar la altura máxima a la frecuencia del loop, no solo cada transmisión.
    // Seguir durante el descenso para no perder nuevos máximos legítimos.
    if (estadoDelCohete >= ascenso && barOK && altura > alturaMaxima)
      alturaMaxima = altura;

    switch (estadoDelCohete) {
      case cohete_desarmado:
        if (nuevaPulsacion && barOK && imuActualValida() && almacenamientoPreparado()) {
          alturaDespegue = altura;
          ciclosAceleracion = 0;
          estadoDelCohete = esperando_despegue;
        }
        break;

      case esperando_despegue:
        if (barOK && aceleracionMayor4G()) {
          // Congelar la referencia desde el primer ciclo de aceleración.
          if (++ciclosAceleracion >= 32) {
            estadoDelCohete = ascenso;
            numeroAlturas = 0;  // Descartar alturas anteriores al vuelo.
            indiceAltura = 0;
            velocidadValida = false;
          }
        } else {
          ciclosAceleracion = 0;
          if (barOK) alturaDespegue = altura;
        }
        break;

      case ascenso:
        if (barOK && altura - alturaDespegue > 100.0f) apogeoHabilitado = true;
        // Exigir 2 m de bajada desde la máxima altura evita falsos apogeos por mesetas.
        if (apogeoHabilitado && barOK && velocidadValida &&
            alturaMaxima - altura >= 2.0f && velocidadVertical <= 0.0f) {
          if (++ciclosApogeo >= CICLOS_CONFIRMAR_APOGEO) {
            estadoDelCohete = drogue_desplegado;
            instanteApogeo = ahora;
          }
        } else ciclosApogeo = 0;
        break;

      case drogue_desplegado:
        // Umbral del principal solo en descenso y con un tiempo mínimo tras el apogeo.
        if (ahora - instanteApogeo >= TIEMPO_MINIMO_DROGUE_MS &&
            barOK && velocidadValida && velocidadVertical < -0.5f &&
            altura - alturaDespegue < alturaMain) {
          if (++ciclosMain >= CICLOS_CONFIRMAR_MAIN)
            estadoDelCohete = paracaidas_desplegado;
        } else ciclosMain = 0;
        break;

      case paracaidas_desplegado:
        // Aterrizaje: velocidad pequeña durante 5 s, con muestras frescas.
        if (barOK && velocidadValida && fabsf(velocidadVertical) < 1.0f) {
          if (inicioEstabilidad == 0) {
            inicioEstabilidad = ahora;
            alturaInicioEstabilidad = altura;
          }
          // Una caída lenta durante 5 s NO equivale a haber aterrizado.
          if (fabsf(altura - alturaInicioEstabilidad) >= 1.0f) {
            inicioEstabilidad = 0;
          } else if (ahora - inicioEstabilidad >= TIEMPO_ESTABILIDAD_ATERRIZAJE_MS)
            estadoDelCohete = aterrizado;
        } else inicioEstabilidad = 0;
        break;

      case aterrizado:
        break;
    }

    actualizarEstadoEnDatos();
    // Estados SOLO lógicos. Los paracaídas se despliegan mecánicamente.
  }
////////////////////////////////////


//////////////////////////////////// Inicialización - se ejecuta una vez al principio. Inicializaciones de los sensores
void setup() {
  // Salidas pirotécnicas inicialmente apagadas; no se activan en este código.
  pinMode(DROGUE, OUTPUT);
  pinMode(PARACAIDAS, OUTPUT);
  digitalWrite(DROGUE, LOW);
  digitalWrite(PARACAIDAS, LOW);

  pinMode(pin_led, OUTPUT);
  pinMode(pin_boton, INPUT);
  pinMode(pin_buzzer, OUTPUT);
  digitalWrite(pin_buzzer, LOW);

  Serial.begin(115200);
  for (int i = 0; i < 50; i++) {
    delay(20);
    digitalWrite(pin_led, HIGH);
    delay(20);
    digitalWrite(pin_led, LOW);
    if (Serial) break;
  }

  // Se conserva la configuración del watchdog del código original.
  NRF_WDT->INTENCLR = 0;
  NRF_WDT->CONFIG = 0;

  iniciarSensores();

  int error = iniciarMemorias();
  if (error != 0) {
    Serial.print("Error inicializando las memorias: ");
    Serial.println(error);
    delay(1000);
  }

  exportarUltimoVueloASD(); // Autoexportar al insertar SD al arrancar, sin borrar Flash

  error = iniciarRadio();
  if (error != RADIOLIB_ERR_NONE) {
    Serial.print("Error inicializando el LoRa: ");
    Serial.println(error);
    delay(1000);
  }

  digitalWrite(pin_led, HIGH); delay(200);
  digitalWrite(pin_led, LOW);  delay(200);
  digitalWrite(pin_led, HIGH); delay(200);
  digitalWrite(pin_led, LOW);

  alturaDespegue = obtenerAlturaBarometrica();
}
////////////////////////////////////



//////////////////////////////////// Bucle de comportamiento - aquí se ejecuta todo lo que hace falta, y se hace con la frecuencia "FRECUENCIA_LOOP_HZ"
void loop() {
  const uint32_t inicioComputo = micros();

  procesarGPS();                 // Decodifica los caracteres pendientes del UART
  leerSensores();                // IMU, magnetómetro, barómetro y último GPS
  comprobarEstadoSensores();     // Test periódico de comunicación y valores
  actualizarMaquinaDeEstados();  // Transiciones de vuelo
  actualizarHistorial();         // RAM: guarda SIEMPRE las últimas 128 muestras
  static bool aterrizajeGuardado = false;
  if (estadoDelCohete >= ascenso) {
    // Flash: registra desde el despegue hasta llenarse, incluso tras el aterrizaje.
    // SD: registra hasta aterrizar y después se cierra para proteger el CSV.
    if (estadoDelCohete < aterrizado) guardarEnMemorias(true, true);
    else if (!aterrizajeGuardado) {
      guardarEnMemorias(true, true);
      finalizarGuardado();
      aterrizajeGuardado = true;
    } else guardarEnMemorias(true, false);
  }
  enviarTelemetriaSiToca();      // Envío LoRa cada ~200 ms
  actualizarIndicadores();       // Botón, LED y buzzer (comportamiento original)
  mostrarDebug(micros() - inicioComputo);
  mostrarAlturaMaximaSerial();

  esperarSiguienteCiclo();       // Periodicidad gestionada por FreeRTOS
}
////////////////////////////////////





//////////////////////////////////// Funciones varias


  // FreeRTOS ya ejecuta loop() en una tarea del core nRF52 de Adafruit.
  // Reparte ticks 20/21, etc. para respetar la frecuencia MEDIA deseada:
  // el tick nRF52 es 1024 Hz y no siempre es divisible por FRECUENCIA_LOOP_HZ.
  void esperarSiguienteCiclo() {
    static TickType_t ultimoDespertar = xTaskGetTickCount();
    static uint32_t restoTicks = 0;
    restoTicks += configTICK_RATE_HZ;
    const TickType_t ticksDeEsteCiclo = restoTicks / FRECUENCIA_LOOP_HZ;
    restoTicks %= FRECUENCIA_LOOP_HZ;
    vTaskDelayUntil(&ultimoDespertar, ticksDeEsteCiclo);
  }

  // Activar el buzzer un momento si se pulsa el botón
  // Para verificar que todo funciona y no se ha crasheado la placa
  void actualizarIndicadores() {
    if (digitalRead(pin_boton) == LOW) {
      digitalWrite(pin_led, HIGH);
      tone(pin_buzzer, 880 * 4);
    } else {
      digitalWrite(pin_led, LOW);
      noTone(pin_buzzer);
    }
  }


////////////////////////////////////



