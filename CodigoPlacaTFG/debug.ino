/* DEPURACIÓN POR SERIAL */
#define cuantosSaltarseDebug 10
#define debugPrintDatos true
#define debugDatosBytes false
#define debugParaSerialPlotter false
#define debugTiempoComputo modoTest
#define debugGPS false
#define debugRecepcionGPS false
#define debugEstadoSensores modoTest  // Diagnóstico resumido cada segundo
#define debuggearValorMedioSensores false

int cont = 0;

//////////////////////////////////// Depuración
  void debuggearTodo(){
    cont++;
    if(cont == cuantosSaltarseDebug){

      #if debugRecepcionGPS
        Serial.print(recepcionGPS);
      #endif

      #if debugPrintDatos
        sprintf(buffer, "%2d-%2d-20%2d %2d:%2d:%2d ACEL:% 2.6f,% 2.6f,% 2.6f GIRO:% 6.2f,% 6.2f,% 6.2f MAG:% 6.2f,% 6.2f,% 6.2f GPS:%f,%f,h:%.2f,v:%.3f,hdop:%.2f,fix:%4d BAR:%4.4f,% 3.2f,h:% 4.2f",
          dSt.dato.fecha[0],dSt.dato.fecha[1],dSt.dato.fecha[2],dSt.dato.tiempo[2],dSt.dato.tiempo[1],dSt.dato.tiempo[0],
          dSt.dato.acc[0],dSt.dato.acc[1],dSt.dato.acc[2],dSt.dato.gyro[0],dSt.dato.gyro[1],dSt.dato.gyro[2],dSt.dato.magn[0],dSt.dato.magn[1],dSt.dato.magn[2],
          /*(dSt.dato.gpsValido)? 1:0,*/ dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad,dSt.dato.hdop,dSt.dato.tiempoDesdeUltimoFix,
          dSt.dato.presion,dSt.dato.temperatura,dSt.dato.altitud_barometro);
        Serial.println(buffer);
      #endif

      #if debugGPS
        sprintf(buffer, "%2d:%2d:%2d %2d-%2d-20%2d , valido=%d, %f,%f,%.2f,%.3f,hdop:%.2f,fix:%4d x:%f, y:%f",
          dSt.dato.tiempo[2],dSt.dato.tiempo[1],dSt.dato.tiempo[0],dSt.dato.fecha[0],dSt.dato.fecha[1],dSt.dato.fecha[2],
          (dSt.dato.gpsValido)? 1:0, dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad,dSt.dato.hdop,dSt.dato.tiempoDesdeUltimoFix,
          (dSt.dato.latitud-41.685455)*111317.099692, (dSt.dato.longitud+0.887576)*111317.099692); //radio_de_la_tierra * 2pi / 360
        Serial.println(buffer);
      #endif

      #if debugDatosBytes
        for(int i=0; i<sizeof(dSt.byte); i++){
          sprintf(buffer, "%3d ", dSt.byte[i]);
          Serial.print(buffer);
        }
        Serial.print("bytes:");
        Serial.println(sizeof(dSt.byte));
      #endif
      
      #if debugParaSerialPlotter
        sprintf(buffer, "Variable_1:%f,Variable_2:%f,Variable_3:%f,Variable_4:%f,Variable_5:%f,Variable_6:%f,Variable_7:%d,Variable_8:%f,Variable_9:%f,Variable_10:%f,Variable_11:%f",
          dSt.dato.acc[0],dSt.dato.acc[1],dSt.dato.acc[2],dSt.dato.gyro[0],dSt.dato.gyro[1],dSt.dato.gyro[2],
          (dSt.dato.gpsValido)? 1:0, dSt.dato.latitud, dSt.dato.longitud, dSt.dato.altitud, dSt.dato.velocidad);
        Serial.println(buffer);
      #endif

      #if debuggearValorMedioSensores
        for(int i=0; i<3; i++){
          mediaAcc[i]  += gananciaParaLaMedia*(dSt.dato.acc[i]-mediaAcc[i]);
          mediaGiro[i] += gananciaParaLaMedia*(dSt.dato.gyro[i]-mediaGiro[i]);
          mediaMagn[i] += gananciaParaLaMedia*(dSt.dato.magn[i]-mediaMagn[i]);
        }
        
        sprintf(buffer, "IMU:% 2.9f,% 2.9f,% 2.9f,% 6.6f,% 6.6f,% 6.6f MAG:% 6.6f,% 6.6f,% 6.6f",
          mediaAcc[0],mediaAcc[1],mediaAcc[2],mediaGiro[0],mediaGiro[1],mediaGiro[2],mediaMagn[0],mediaMagn[1],mediaMagn[2]);
        Serial.println(buffer);
      #endif

      cont = 0;
    }
  }

  // Debe llamarse al final de cada ciclo: conserva la impresión original
  // de tiempo de cómputo incluso con modoTest desactivado.
  void mostrarDebug(uint32_t tiempoComputoUs) {
    #if modoTest
      debuggearTodo();
    #endif
    #if debugEstadoSensores
      mostrarDiagnosticoSensores();
    #endif
    #if debugTiempoComputo
      static uint32_t ultimoDebugTiempo = 0;
      if (millis() - ultimoDebugTiempo >= 1000) {
        ultimoDebugTiempo = millis();
        Serial.print("Tiempo computo (us):");
        Serial.println(tiempoComputoUs);
      }
    #endif
  }

  void mostrarAlturaMaximaSerial() {
    float alturaRelativa = 0.0f;

    if (estadoDelCohete >= ascenso &&
      isfinite(alturaMaxima) &&
      isfinite(alturaDespegue) &&
      alturaMaxima > -99999.0f) {
      alturaRelativa = fmaxf(0.0f, alturaMaxima - alturaDespegue);
    }
    
    Serial.print("ALTURA_MAXIMA_M: ");
    Serial.println(alturaRelativa, 2);
  }
////////////////////////////////////



  void mostrarDiagnosticoSensores(){
    static uint32_t ultimoMensaje = 0;
    if(millis() - ultimoMensaje < 1000) return;
    ultimoMensaje = millis();

    Serial.print("[SALUD] IMU:");
    Serial.print((flagsSensoresActivos & IMUfuncionando) ? "OK" : "ERROR");
    Serial.print(" ID:0x"); Serial.print(idIMU, HEX);
    Serial.print(" LECT:"); Serial.print(ultimaLecturaIMUValida ? "OK" : "ERROR");
    Serial.print(" FALLOS:"); Serial.print(erroresLecturaIMU);
    Serial.print(" IMU_SIN_MUESTRA:"); Serial.print(muestrasIMUSinActualizar);
    Serial.print(" BAR_FRESCO:"); Serial.print(barLecturaFresca ? "SI" : "NO");
    Serial.print(" FLASH_S:"); Serial.print(segundosFlashDisponibles());
    Serial.print(" MAG:");
    Serial.print((flagsSensoresActivos & MAGfuncionando) ? "OK" : "ERROR");
    Serial.print(" BAR:");
    Serial.print((flagsSensoresActivos & BARfuncionando) ? "OK" : "ERROR");
    Serial.print(" GPS_NMEA:");
    Serial.print(gpsComunicando ? "OK" : "SIN DATOS");
    Serial.print(" GPS_FIX:");
    Serial.print((flagsSensoresActivos & tfixGPSmenorA200ms) ? "OK" : "SIN FIX RECIENTE");
    Serial.print(" GPS_HDOP:");
    Serial.print((flagsSensoresActivos & hdopGPSmenorA5m) ? "OK" : "ALTO/NO FIX");
    Serial.print(" SD:");
    Serial.print((flagsSensoresActivos & SDfuncionando) ? "OK" : "ERROR");
    Serial.print(" FLASH:");
    Serial.print((flagsSensoresActivos & FLASHfuncionando) ? "OK" : "ERROR");
    Serial.print(" FLAGS:0x");
    Serial.println(flagsSensoresActivos, HEX);
  }
////////////////////////////////////
