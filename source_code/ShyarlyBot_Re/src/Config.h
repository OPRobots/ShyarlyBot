/**
 * @file Config.h
 * @author Álex Santos (@robotaleh)
 * @brief Fichero de configuración
 * @version 0.1
 * @date 2019-04-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <Constantes.h>

#define SERIAL_BAUDRATE 9600

/**
 * @brief Establece el tipo de pista en la que va a correr el robot
 * 
 * LINEA_NEGRA | LINEA_BLANCA
 * 
 */
#define TIPO_LINEA LINEA_BLANCA

/**
 * @brief Tiempo (ms) durante el que se realiza la calibración de sensores 
 * 
 */
#define TIEMPO_CALIBRACION 5000

/**
 * @brief Establece el número de sensores del robot
 * 
 */
#define NUMERO_SENSORES 15

/**
 * @brief Establece el número de Switches para estrategias
 * 
 */
#define NUMERO_SWITCHES 3

/**
 * @brief Offset aplicado en el cálculo del Umbral de Saturación para "inclinarlo hacia línea"
 * 
 */
#define GANANCIA_UMBRAL 100

/**
 * @brief Establece el abanico de Analógico de los sensores "hacia blanco" o "hacia línea"
 * 
 * Nota: Solo admite valores positivos
 * 
 */
#define UMBRAL_BLANCO 0
#define UMBRAL_NEGRO 0
