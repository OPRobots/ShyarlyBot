/**
 * @file Calibracion.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene todas las funciones de Calibración y cálculo de Umbrales para los sensores
 * @version 0.1
 * @date 2019-04-23
 *
 * @copyright Copyright (c) 2019
 *
 */

/**
 * @brief Calibra los sensores durante TIEMPO_CALIBRACION y calcula los Umbrales de saturación
 *
 */
void realizar_calibracion() {
  digitalWrite(PIN_LED, HIGH);
  calibrar_sensores();
  calcular_umbrales();
  digitalWrite(PIN_LED, LOW);
}

/**
 * @brief Calibra los sensores
 * Durante TIEMPO_CALIBRACION, toma los valores máximos y mínimos de cada sensor y los guarda en sus respectivos arrays para linealizar los valores en las lecturas
 *
 */
void calibrar_sensores() {
  long millisInicio = millis();
  do {
    for (int sensor = 0; sensor < NUMERO_SENSORES; sensor++) {
      valoresSensores[sensor] = muxAnalogRead(sensorPins[sensor]);
      if (valoresSensores[sensor] > valoresSensoresMaximos[sensor]) {
        valoresSensoresMaximos[sensor] = valoresSensores[sensor];
      }
      if (valoresSensores[sensor] < valoresSensoresMinimos[sensor]) {
        valoresSensoresMinimos[sensor] = valoresSensores[sensor];
      }
    }
  } while ((millis() - millisInicio) <= TIEMPO_CALIBRACION);
}

/**
 * @brief Calcula los Umbrales de Saturación de cada sensor
 * El Umbral de cada sensor está definido por el valor medio de los valores máximos y mínimos de sus sensores y una Ganancia aplicada para inclinar el Umbral "hacia línea"
 * 
 */
void calcular_umbrales() {
  for (int sensor = 0; sensor < NUMERO_SENSORES; sensor++) {
    umbralesSensores[sensor] = ((valoresSensoresMaximos[sensor] + valoresSensoresMinimos[sensor]) / 2) + GANANCIA_UMBRAL;
    umbralesSensoresMapeados[sensor] = map(umbralesSensores[sensor], valoresSensoresMinimos[sensor], valoresSensoresMaximos[sensor], 0, 255);
  }

  // TODO: pasar prints a fichero de Debug
  /*int i;
   for (i = 0; i < NUMERO_SENSORES; i++) {
    Serial.print(umbralesSensoresMapeados[i]);
    Serial.print("\t");
  }
  Serial.println("\t");
  for (i = 0; i < NUMERO_SENSORES; i++) {
    Serial.print(valoresSensoresMaximos[i]);
    Serial.print("\t");
  }
  Serial.println("\t");
  for (i = 0; i < NUMERO_SENSORES; i++) {
    Serial.print(valoresSensoresMinimos[i]);
    Serial.print("\t");
  }
  Serial.println("\t"); */
}