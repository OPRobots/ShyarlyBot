/**
 * @file Pinout.h
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene el pinout principal
 * @version 0.1
 * @date 2019-04-23
 *
 * @copyright Copyright (c) 2019
 *
 */

/*
  =========================================
                PINES SENSORES
  =========================================
*/
// De izquierda a derecha:
#define SENSOR_1 0
#define SENSOR_2 1
#define SENSOR_3 2
#define SENSOR_4 3
#define SENSOR_5 4
#define SENSOR_6 5
#define SENSOR_7 6
#define SENSOR_8 7
#define SENSOR_9 8
#define SENSOR_10 9
#define SENSOR_11 10
#define SENSOR_12 11
#define SENSOR_13 12
#define SENSOR_14 13
#define SENSOR_15 14

#define MULTIPLEXADOR_SIGNAL A1
#define MULTIPLEXADOR_CH_0 A5
#define MULTIPLEXADOR_CH_1 A4
#define MULTIPLEXADOR_CH_2 A3
#define MULTIPLEXADOR_CH_3 A2

// Pines de los motores
#define MOTOR_IZQUIERDO_ADELANTE 6
#define MOTOR_IZQUIERDO_ATRAS 7
#define MOTOR_DERECHO_ADELANTE 9
#define MOTOR_DERECHO_ATRAS 10
#define MOTOR_DERECHO_PWM 11
#define MOTOR_IZQUIERDO_PWM 5

#define MOTORES_STANDBY 8

// Pines de los Sensores lellendolos de izquierda a derecha viendo el robot de
// la bateria a los sensores int sensores[] = {A7, A6, A3, A2, A1};

// Pines de los interruptores para diferentes modos de velocidad
#define BTN 12
#define PIN_LED 13
int switches[] = {2, 3, 4};