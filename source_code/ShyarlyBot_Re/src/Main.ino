/**
 * @file Main.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero principal del programa, que contiene las funciones principales; setup() y loop()
 * @version 0.1
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <PIDfromBT.h>

#include <Config.h>

#include <Pinout.h>

int valoresSensoresMaximos[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int valoresSensoresMinimos[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int umbralesSensores[15];
int umbralesSensoresMapeados[15];

int sensorPins[] = {SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8, SENSOR_9, SENSOR_10, SENSOR_11, SENSOR_12, SENSOR_13, SENSOR_14, SENSOR_15};

long valoresSensores[NUMERO_SENSORES];

// Variables PID
float kp = 0;
float kd = 0;
float ki = 0;
int posicion = 0;
int errorAnterior = 0;
int correccion = 0;

bool arranque = false;
long millisInicio = 0;
long millisAnteriorPID = 0;

int posicionIdeal = 0;
int posicionMaxima = (NUMERO_SENSORES + 1) * (1000 / 2);
int posicionMaximaMapeada = 500;
int posicionMinimaMapeada = -500;

int velocidad = 0;

PIDfromBT pid_calibrate(&kp, &ki, &kd, &velocidad, DEBUG);

void setup() {
  iniciar_todo();
  realizar_calibracion();
}

void loop() {
  pid_calibrate.update();

  if (digitalRead(BTN) == false) {
    digitalWrite(PIN_LED, HIGH);
    arranque = true;
    millisInicio = millis();
    establecer_estrategia(lectura_estrategia());
  } else if ((millis() >= (millisInicio + 5000)) && (arranque == true) && (millisInicio > 0)) {

    if (millis() >= millisAnteriorPID + 1) {

      lectura_sensores_mapeados(true);
      posicion = calcular_posicion_linea(posicion);
    //   Serial.println(posicion);
      correccion = calcular_correccion_PID(posicion);
      asignar_velocidad(velocidad, correccion);

      millisAnteriorPID = millis();
    }
  } else {
    digitalWrite(PIN_LED, LOW);
    millisAnteriorPID = 0;
  }
}
