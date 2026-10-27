/*
  AVIONICA - PROGRAMA PRINCIPAL (nRF52 / Adafruit Arduino Core)

  Solo coordina la placa. Las funciones de sensores y comunicaciones
  se implementan en las otras dos pestañas .ino.

  IMPORTANTE: el código original solo declaraba los estados; aún NO
  implementa las transiciones ni el disparo de los paracaídas.
*/
struct ConfiguracionLoRa; // Adelanto para los prototipos autogenerados de Arduino.

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <FreeRTOS.h>
#include <task.h>
#include <RadioLib.h>       // Constante RADIOLIB_ERR_NONE

//////////////////////////////////// Ajustes que hay que revisar antes de cada lanzamiento
  #define FRECUENCIA_LOOP_HZ 100
            // Frecuencia objetivo (compilación); antes: 20 ms

  #define modoTest readTemperature
            // AVISO: revisar antes del vuelo

  #define milisegundosAntesDeApogeo 1000
            // Esto será para poder despegar el paracaídas antes o después del apogeo
            // TODAVÍA NO ESTÁ IMPLEMENTADO

  #define alturaMain 150
            // m sobre altura de despegue
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



//////////////////////////////////// Máquina de estados
  enum estados {
    cohete_desarmado,
    inicio,
    esperando_despegue,
    ascenso,
    drogue_desplegado,
    paracaidas_desplegado,
    aterrizado
  };

  char estadoDelCohete = cohete_desarmado;

  // Referencias utilizadas para enviar telemetría relativa al despegue.
  float alturaDespegue = 220.0f;
  float latitudDespegue = 41.685109f;    // Mantener valores originales por ahora
  float longitudDespegue = -0.887776f;  // TODO: obtener de GPS antes de despegue

  void actualizarMaquinaDeEstados() {
    // No había transiciones implementadas en el archivo original.
    // Completar aquí las condiciones de cada estado antes de utilizarlo en vuelo.
    switch (estadoDelCohete) {
      case cohete_desarmado:      break;
      case inicio:                break;
      case esperando_despegue:               
        deteccionApogeo(); // Esto tan solo se ejecuta para ir llenando la memoria con la altura actual
        break;
      case ascenso:               
        bool apogeo = deteccionApogeo();
        break;
      case drogue_desplegado:     break;
      case paracaidas_desplegado: break;
      case aterrizado:            break;
    }
  }

  // Esta función va grabando constantemente la altura leída para poder detectar el punto de apogeo
  // Devuelve cuando estamos en el punto de apogeo y más allá. Se debe ejecutar en cada ciclo
  float arrayAlturas[128];
  int punteroArrayAlturas = 0;
  bool deteccionApogeo(){
    arrayAlturas[punteroArrayAlturas] = dSt.dato.altitud_barometro;
    punteroArrayAlturas = (punteroArrayAlturas++) & 0x7F;
    bool velocidadDeApogeo = 
          arrayAlturas[punteroArrayAlturas] - arrayAlturas[(punteroArrayAlturas-1) & 0x7F] 
                <= 
          pow(  4.95 * ( (float)(periodoMuestreo*128)/1000.0 )   ,  2.0);
    bool estamosEnMovimiento = 
          arrayAlturas[punteroArrayAlturas] - arrayAlturas[(punteroArrayAlturas-1) % 128] 
                >= 
          pow(  2.475 * ( (float)(periodoMuestreo*128)/1000.0 )   ,   2.0);
    return velocidadDeApogeo && estamosEnMovimiento;
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
  actualizarMaquinaDeEstados();  // Pendiente de implementar transiciones
  guardarEnMemorias(false, true);// Flash sin implementar; SD activada
  enviarTelemetriaSiToca();      // Envío LoRa cada ~200 ms
  actualizarIndicadores();       // Botón, LED y buzzer (comportamiento original)
  mostrarDebug(micros() - inicioComputo);

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



