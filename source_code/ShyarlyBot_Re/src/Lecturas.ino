/**
 * @file Lecturas.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene todas las funciones relacionadas con las Lecturas y el manejo de los Sensores
 * @version 0.1
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
 * @brief Lectura de sensores desde Multiplexador
 * Establece los canales del Multiplexador en función del sensor que se quiere
 * leer y retorna el valor analógico del sensor
 *
 * @param sensor Número del sensor, del 0 al 15 que se quiere leer.
 * @return short Valor analógico del sensor
 */

short muxAnalogRead(int sensor) {
  digitalWrite(MULTIPLEXADOR_CH_0, bitRead(sensor, 0));
  digitalWrite(MULTIPLEXADOR_CH_1, bitRead(sensor, 1));
  digitalWrite(MULTIPLEXADOR_CH_2, bitRead(sensor, 2));
  digitalWrite(MULTIPLEXADOR_CH_3, bitRead(sensor, 3));
  if (TIPO_LINEA == LINEA_NEGRA) {
    return 1023 - analogRead(MULTIPLEXADOR_SIGNAL);
  } else {
    return analogRead(MULTIPLEXADOR_SIGNAL);
  }
}

void lectura_sensores() {
  for (int sensor = 0; sensor < NUMERO_SENSORES; sensor++) {
    valoresSensores[sensor] = muxAnalogRead(sensorPins[sensor]);
  }
}

void lectura_sensores_mapeados(bool saturarSensores) {
  for (int sensor = 0; sensor < NUMERO_SENSORES; sensor++) {
    valoresSensores[sensor] = map(muxAnalogRead(sensorPins[sensor]), valoresSensoresMinimos[sensor], valoresSensoresMaximos[sensor], 0, 255);
    if (saturarSensores) {
      if (valoresSensores[sensor] < umbralesSensoresMapeados[sensor] - UMBRAL_BLANCO) {
        valoresSensores[sensor] = 0;
      } else if (valoresSensores[sensor] > umbralesSensoresMapeados[sensor] + UMBRAL_NEGRO) {
        valoresSensores[sensor] = 255;
      }
    }
  }
}
