/**
 * @file Control.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene las funciones de Control y Movimiento del robot
 * @version 0.1
 * @date 2019-04-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */

int calcular_posicion_linea(int posicionAnterior) {
  unsigned long sumaSensoresPonderados = 0;
  unsigned long sumaSensores = 0;
  int sensoresEnLinea = 0;
  int posicionActual = 0;
  for (int sensor = 0; sensor < NUMERO_SENSORES; sensor++) {
    //   Serial.print(valoresSensores[sensor]);
    //   Serial.print("\t");
    sumaSensoresPonderados = sumaSensoresPonderados + ((sensor + 1) * valoresSensores[sensor] * 1000L);
    sumaSensores = sumaSensores + valoresSensores[sensor];
    if (valoresSensores[sensor] > umbralesSensoresMapeados[sensor]) {
      sensoresEnLinea = sensoresEnLinea + 1;
    }

  }
//   Serial.print(sensoresEnLinea);
//   Serial.println();

  if (sensoresEnLinea > 0 && sensoresEnLinea != NUMERO_SENSORES) {
    // TODO: Actualizar millis de ultima vez en línea
  }

  if (sensoresEnLinea > 0) {
    posicionActual = ((sumaSensoresPonderados / sumaSensores) - posicionMaxima);
  } else if (posicionAnterior > 0) {
    posicionActual = posicionMaxima;
  } else {
    posicionActual = -posicionMaxima;
  }
//   Serial.println(posicionActual);
  return map(posicionActual, -posicionMaxima, posicionMaxima, posicionMinimaMapeada, posicionMaximaMapeada);
  
}

int calcular_correccion_PID(int posicionActual) {
  float p = 0;
  float i = 0;
  float d = 0;
  int error = posicionIdeal - posicionActual;

  p = kp * error;
  d = kd * (error - errorAnterior);
  errorAnterior = error;

  return map(p + i + d, -10000, 10000, -1000, 1000);
}

void asignar_velocidad(int velocidad, int correccion) {
  int velD = velocidad + correccion;
  int velI = velocidad - correccion;
// Serial.println(velocidad);
  //de tal modo que si la correccion hace que una rueda se ponga a mas de 255, se limita a 255 y a la otra se le aplique la correccion restante
  if (velD > 255) {
    velI = velI - (velD - 255);
    velD = 255;
  }
  if (velI > 255) {
    velD = velD - (velI - 255);
    velI = 255;
  }

  //limitamos desbordamientos
  velD = constrain(velD, -255, 255);
  velI = constrain(velI, -255, 255);

  if (velD >= 0) {
    digitalWrite(MOTOR_DERECHO_ADELANTE, HIGH);
    digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
    analogWrite(MOTOR_DERECHO_PWM, velD);
  } else {
    digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
    digitalWrite(MOTOR_DERECHO_ATRAS, HIGH);
    analogWrite(MOTOR_DERECHO_PWM, abs(velD));
  }

  if (velI >= 0) {
    digitalWrite(MOTOR_IZQUIERDO_ADELANTE, HIGH);
    digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);
    analogWrite(MOTOR_IZQUIERDO_PWM, velI);
  } else {
    digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
    digitalWrite(MOTOR_IZQUIERDO_ATRAS, HIGH);
    analogWrite(MOTOR_IZQUIERDO_PWM, abs(velI));
  }
}

void parada_emergencia() {
  velocidad = 0;
  kp = 0;
  ki = 0;
  kd = 0;
}