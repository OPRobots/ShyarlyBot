/**
 * @file Estrategias.ino
 * @author Álex Santos (@robotaleh)
 * @brief Fichero que contiene la lectura y el manejo de las Estrategias
 * @version 0.1
 * @date 2019-04-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
 * @brief Obtiene la Estrategia seleccionada
 * Calcula la Estrategia determinada por los switches en binario
 * 
 * @return short Estrategia calculada
 */
short lectura_estrategia() {
  int binario = 1;
  int estrategia = 0;
  int lectura_switch[] = {0, 0, 0};
  for (int sw = NUMERO_SWITCHES - 1; sw >= 0; sw--) {
    lectura_switch[sw] = !digitalRead(switches[sw]);
    estrategia = estrategia + (lectura_switch[sw] * binario);
    binario = binario * 2;
  }
  Serial.println(estrategia);
  return estrategia;
}

/**
 * @brief Actualiza los valores de Velocidad y PID según la Estrategia
 * 
 * @param estrategia 
 */
void establecer_estrategia(short estrategia) {
  int velocidadPorcentaje = 0;
  switch (estrategia) {

  case 0:
    velocidadPorcentaje = 0; //Escala 0 - 100%  --  PWM 51
    kp = 5;
    kd = 80;
    break;

  case 1:
    velocidadPorcentaje = 35; //Escala 0 - 100%  --  PWM 82.85
    kp = 5;
    kd = 80;
    break;

  case 2:
    velocidadPorcentaje = 40; //Escala 0 - 100%  --  PWM 89.25
    kp = 5;
    kd = 80;
    break;

  case 3:
    velocidadPorcentaje = 45; //Escala 0 - 100%  --  PWM 95.6
    kp = 5;
    kd = 80;
    break;

  case 4:
    velocidadPorcentaje = 50; //Escala 0 - 100%  --  PWM 102
    kp = 5;
    kd = 80;
    break;

  case 5:
    velocidadPorcentaje = 55; //Escala 0 - 100%  --  PWM 108
    kp = 5;
    kd = 80;
    break;

  case 6:
    velocidadPorcentaje = 60; //Escala 0 - 100%  --  PWM 114.75
    kp = 5;
    kd = 80;
    break;

  case 7:
    velocidadPorcentaje = 65; //Escala 0 - 100%  --  PWM 121
    kp = 5;
    kd = 80;
    break;

  case 8:
    velocidadPorcentaje = 70; //Escala 0 - 100%  --  PWM 127.5
    kp = 5;
    kd = 80;
    break;
  }
  velocidad = map(velocidadPorcentaje, 0, 100, 0, 255);
}