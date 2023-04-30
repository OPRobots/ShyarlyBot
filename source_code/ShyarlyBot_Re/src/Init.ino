/**
 * @file Init.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene todas las inicializaciones de Pines y Periféricos
 * @version 0.1
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

void iniciar_todo() {
  iniciar_serial();
  iniciar_sensores();
  iniciar_auxiliares();
  iniciar_sensores();
  iniciar_driver_motores();
}

void iniciar_serial() {
  Serial.begin(SERIAL_BAUDRATE);
}

void iniciar_auxiliares() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  for (int sw = 0; sw < NUMERO_SWITCHES; sw++) {
    pinMode(switches[sw], INPUT_PULLUP);
  }
}

void iniciar_sensores() {
  pinMode(MULTIPLEXADOR_SIGNAL, INPUT);
  pinMode(MULTIPLEXADOR_CH_0, OUTPUT);
  pinMode(MULTIPLEXADOR_CH_1, OUTPUT);
  pinMode(MULTIPLEXADOR_CH_2, OUTPUT);
  pinMode(MULTIPLEXADOR_CH_3, OUTPUT);
}

void iniciar_driver_motores() {
  pinMode(MOTOR_IZQUIERDO_ADELANTE, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_ATRAS, OUTPUT);
  pinMode(MOTOR_DERECHO_ADELANTE, OUTPUT);
  pinMode(MOTOR_DERECHO_ATRAS, OUTPUT);
  pinMode(MOTORES_STANDBY, OUTPUT);
  digitalWrite(MOTORES_STANDBY, HIGH);

  digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
  digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);
  digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
  digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
}
