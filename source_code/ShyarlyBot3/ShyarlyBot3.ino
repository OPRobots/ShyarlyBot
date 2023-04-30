
#include <PIDfromBT.h> // Calibracion de BT desde App Android PIDfromBT
// #include <SoftwareSerial.h>
/*
	Robot Siguelineas: ShyarlyBot, creado y diseñado íntegramente por OP-Robots.
	Codigo desarrollado por AlexSantos para uso exclusivo del robot anteriormente mencionado.

	Created at: 2017-04-24 09:26:45
	Created by: Alex Santos

*/

/*
  =========================================
                LIBRERÍAS
  =========================================
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

#define muxSIG A1
#define muxS0 A5
#define muxS1 A4
#define muxS2 A3
#define muxS3 A2

/*
  =========================================
                PINES MOTORES
  =========================================
*/
#define MOTOR_DERECHO_ADELANTE	    9
#define MOTOR_DERECHO_ATRAS		    10
#define MOTOR_DERECHO_PWM		    11
#define MOTOR_IZQUIERDO_ADELANTE    6
#define MOTOR_IZQUIERDO_ATRAS 	    7
#define MOTOR_IZQUIERDO_PWM		  	5
#define MOTOR_STBY 					8
#define MOTOR_DERECHO_OFFSET		0
#define MOTOR_IZQUIERDO_OFFSET		3

/*
  =========================================
                PINES BOTÓN/SWITCH
  =========================================
*/
#define SW_RASTREADOR 2
#define SW_CIR 3
#define SW_VEL 4
#define BTN_RACE 12

/*
  =========================================
            VARIABLES NEUTRAS
  =========================================
*/
#define CALIBRATION_TIME 5000
int velBase = 0;
int velMax = 255;
float velI = 0;
float velD = 0;
bool run = false;
bool competicion = false;
bool competicion_iniciada = false;
long last_detected_line = 0;

/*
  =========================================
          VARIABLES DE CALIBRADO
  =========================================
*/
const int LED_DERECHO = 13;
const int LED_IZQUIERDO = A0;
float analogico = 0;
int ideal = 0;
float velMod = 0;
float lastError = 0;
long lastMillis = 0;
float sumError = 0;

float kp = 0, ki = 0, kd = 0;

/*
  =========================================
            VARIABLES ACELERACIONES
  =========================================
*/
bool acelerar = false;
float aumentoP = 0.02;
float aumentoD = 0.05;
float maxAumento = 0.5;
float velIncremento = 0.05;
float maxError = 30;

/*
  =========================================
            VARIABLES SENSORES
  =========================================
*/
const int NUM_SENSORS =  15;
int sensorPins[] = {SENSOR_1,
                    SENSOR_2,
                    SENSOR_3,
                    SENSOR_4,
                    SENSOR_5,
                    SENSOR_6,
                    SENSOR_7,
                    SENSOR_8,
                    SENSOR_9,
                    SENSOR_10,
                    SENSOR_11,
                    SENSOR_12,
                    SENSOR_13,
                    SENSOR_14,
                    SENSOR_15
                   };
short sensorWeights[] = { -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7};
short maxAnalog = (abs(sensorWeights[0]) + 1) * 255;
short maxMappedAnalog = 255;
short sensorValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int minVal = 20; // Valor mínimo por debajo del cual se interpreta como línea
int maxVal = 140; // Valor máximo por encima del cual se interpreta como blanco
int minVals[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int maxVals[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// int minVals[] = {287, 197, 288, 248, 251, 240, 195, 210, 260, 279, 225, 253, 201, 269, 229};
// int maxVals[] = {992, 983, 991, 988, 987, 987, 985, 988, 993, 991, 985, 989, 979, 990, 986};



/*
  =========================================
            VARIABLES RASTREADOR
  =========================================
*/
bool marcaDerecha = false;
bool marcaIzquierda = false;
bool marcaDerechaFin = false;
bool marcaIzquierdaFin = false;
bool cegarDerecha = false;
bool cegarIzquierda = false;
bool pasandoInterseccion = false;
bool stateLed = LOW;
long lastStateLed;


/*
  =========================================
          VARIABLES DE ACELERACIÓN
  =========================================
*/
short velReal = 0;
short vel_ini = 0;
long millis_inicial_accel = 0;
float m;
bool cambiando_vel = false;

/*
  =========================================
            ZONA DEBUG
  =========================================
*/
#define ENABLE_GENERAL_DEBUG 1
#define ENABLE_CALIBRATION_DEBUG 0
#define ENABLE_VEL_DEBUG 0
#define ENABLE_ANALOG_READ_DEBUG 0
#define ENABLE_ANALOG_PARSING_DEBUG 0
#define ENABLE_ANALOG_DEBUG 0

#define CALIBRATION_DEBUG 1
#define VEL_DEBUG 2
#define READ_LINE_DEBUG 3
#define ANALOG_PARSING_DEBUG 4
#define ANALOG_DEBUG 5


PIDfromBT pid_calibrate(&kp, &ki, &kd, &velBase, &ideal, DEBUG);
void setup() {

	Serial.begin(9600);

	//	Declarar pines
	pins_init();
	if (!digitalRead(BTN_RACE)) {
		competicion = true;
	}
	// Calibrando sensores
	sensor_calibrate();
 pid_calibrate.setMinIdeal(-8000);
 pid_calibrate.setMaxIdeal(8000);

  kp = 1.7f;
  kd = 150;
  velBase = 120;
}

void loop() {
	if (!competicion || (competicion && competicion_iniciada)) {
		if (!competicion) {
			pid_calibrate.update();
		}

		analogico = read_line(analogico);
		float correccion = calc_PID(analogico);

		//calc_accel(velBase, false, false, 250);
		set_speed(correccion);
		// delay(1);
	} else if (competicion) {
		if (digitalRead(BTN_RACE)) {
			delay(100);
			if (!digitalRead(BTN_RACE)) {
				while (!digitalRead(BTN_RACE)) {
					digitalWrite(LED_DERECHO, HIGH);
				}
				long millis_pre_start = millis();
				bool led_state = true;
				while (millis() < (millis_pre_start + 5000)) {
					if ((millis() - millis_pre_start) % 500 == 0) {
						led_state = !led_state;
						digitalWrite(LED_DERECHO, led_state);
					}
				}
				digitalWrite(LED_DERECHO, LOW);
				competicion_iniciada = true;
			}
		}

	}


}

void pins_init() {
	Serial.begin(9600);
	// Declarar pin LED
	pinMode(LED_DERECHO, OUTPUT);
	pinMode(LED_IZQUIERDO, OUTPUT);

	// Iniciar pines BTN/SW
	pinMode(SW_RASTREADOR, INPUT_PULLUP);
	pinMode(SW_CIR, INPUT_PULLUP);
	pinMode(SW_VEL, INPUT_PULLUP);
	pinMode(BTN_RACE, INPUT_PULLUP);

	//	Declarar pines motores
	pinMode(MOTOR_DERECHO_ADELANTE   , OUTPUT);
	pinMode(MOTOR_DERECHO_ATRAS      , OUTPUT);
	pinMode(MOTOR_DERECHO_PWM        , OUTPUT);
	pinMode(MOTOR_IZQUIERDO_ADELANTE , OUTPUT);
	pinMode(MOTOR_IZQUIERDO_ATRAS	 , OUTPUT);
	pinMode(MOTOR_IZQUIERDO_PWM	     , OUTPUT);
	pinMode(MOTOR_STBY	             , OUTPUT);

	//	Inicializa los motores a estado parado
	digitalWrite(MOTOR_DERECHO_ADELANTE   , LOW);
	digitalWrite(MOTOR_DERECHO_ATRAS      , LOW);
	digitalWrite(MOTOR_IZQUIERDO_ADELANTE , LOW);
	digitalWrite(MOTOR_IZQUIERDO_ATRAS    , LOW);
	digitalWrite(MOTOR_STBY    , LOW);

	// Declara pines sensores
	/*for (int i = 0; i < NUM_SENSORS; i++) {
		pinMode(sensorPins[i], INPUT);
	}*/
	pinMode(muxS0, OUTPUT);
	pinMode(muxS1, OUTPUT);
	pinMode(muxS2, OUTPUT);
	pinMode(muxS3, OUTPUT);
}

void sensor_calibrate() {
	long time = millis();

	bool direccion = true;
	short cont = 0;

	digitalWrite(LED_DERECHO, HIGH);
	digitalWrite(LED_IZQUIERDO, HIGH);
	while (millis() < (time + CALIBRATION_TIME)) {
		for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
			short value = muxAnalogRead(sensorPins[sensor]);
			if (value < minVals[sensor]) {
				minVals[sensor] = value;
			} else if (value > maxVals[sensor]) {
				maxVals[sensor] = value;
			}
		}

		// if ((millis() - time) % 500 == 0 && cont > 0) {
		// 	direccion = !direccion;
		// 	cont++;
		// } else if ((millis() - time) % 250 == 0 && cont == 0) {
		// 	direccion = !direccion;
		// 	cont = cont++;
		// }

		// move_calibrate(time, direccion, cont);
	}
	if (run) {
		run = false;
		digitalWrite(MOTOR_STBY, LOW);
	}
	for (int i = 0; i < NUM_SENSORS; i++) {
		Serial.print(minVals[i]);
		Serial.print(", ");
	}
	Serial.print(" min\n");
	for (int i = 0; i < NUM_SENSORS; i++) {
		Serial.print(maxVals[i]);
		Serial.print(", ");
	}
	Serial.print(" máx\n");
	digitalWrite(LED_DERECHO, LOW);
	digitalWrite(LED_IZQUIERDO, LOW);
	console_debug(CALIBRATION_DEBUG);
	delay(1000);
}

short muxAnalogRead(byte mux_channel) {
	digitalWrite(muxS0, bitRead(mux_channel, 0));
	digitalWrite(muxS1, bitRead(mux_channel, 1));
	digitalWrite(muxS2, bitRead(mux_channel, 2));
	digitalWrite(muxS3, bitRead(mux_channel, 3));
	return analogRead(muxSIG);
}

void move_calibrate(long time, bool direccion, short cont) {

	int pinD = -1;
	int pinI = -1;

	if (direccion) { // Derehca
		pinD = MOTOR_DERECHO_ATRAS;
		pinI = MOTOR_IZQUIERDO_ADELANTE;
	} else { // Izquierda
		pinD = MOTOR_DERECHO_ADELANTE;
		pinI = MOTOR_IZQUIERDO_ATRAS;
	}
	if (!run) {
		run = true;
		digitalWrite(MOTOR_STBY, HIGH);
	}
	digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
	digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
	digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
	digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);

	digitalWrite(pinD, HIGH);
	digitalWrite(pinI, HIGH);

	analogWrite(MOTOR_DERECHO_PWM   , 35);
	analogWrite(MOTOR_IZQUIERDO_PWM , 35);

}

float read_line(float last_analog) {

	// Lectura de sensores
	int lineSensors = (int)NUM_SENSORS;

	bool sensores_detectando[NUM_SENSORS];

	for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
		// Establece inicialmente el sensor como detectando
		sensores_detectando[sensor] = true;

		// Mapea la lectura de los sensores a 0-255, en función de sus maximos y minimos de cada sensor
		sensorValues[sensor] = map(muxAnalogRead(sensorPins[sensor]), minVals[sensor], maxVals[sensor], 0, 255);
		// Satura las lecutras a 0 o 255 en funcion de los valores maximos y minimos definidos
		if (sensorValues[sensor] > maxVal) {
			sensorValues[sensor] = 255;
			lineSensors--;
			// Sobreescribe el estado del sensor como NO detectando
			sensores_detectando[sensor] = false;
		} else if (sensorValues[sensor] < minVal) {
			sensorValues[sensor] = 0;
		}
		sensorValues[sensor] = 255 - sensorValues[sensor];
	}

	rastreador(sensores_detectando, last_analog);

	// Variables para generación de posición
	unsigned long linea = 0;
	unsigned long valores = 0;
	float posicion_real = last_analog;

	// Cálculo de la posición sobre la línea
	for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
		linea += (sensor + 1) * sensorValues[sensor] * 1000L;
		valores += (long)sensorValues[sensor];
	}
	if (lineSensors > 0) {
		posicion_real = ((linea / valores) - ((NUM_SENSORS + 1) * (float)(1000 / 2)));
		// Serial.println(posicion_real);
		return posicion_real;
	} else {
		return (last_analog > 0) ? (1000 * (NUM_SENSORS + 1) / 2) : -(1000 * (NUM_SENSORS + 1) / 2);
	}


	//console_debug(READ_LINE_DEBUG);
}

int num_lineas = 0;
int last_inicial = -1;
void rastreador(bool sensores_detectando[], float last_analog) {
	if (digitalRead(SW_RASTREADOR)) {
		bool m_derecha = false;
		bool m_izquierda = false;
		byte led_state_time = 250;

		int num_lineas_anterior = num_lineas;
		num_lineas = 0;
		bool anterior = false;
		int inicio_linea_principal = last_inicial >= 0 ? last_inicial : (map(last_analog, -7000, 7000, 0, 14));
		int fin_linea_principal = 0;
		bool es_centro = false;
		if (!pasandoInterseccion) {
			for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
				// Serial.print(sensores_detectando[sensor]);
				// Serial.print("\t");


				if (anterior == false && sensores_detectando[sensor]) {
					num_lineas++;
					if (abs(inicio_linea_principal - sensor) < 3) { //TODO: Se puede pasar a == 1? De momento así va bien.
						inicio_linea_principal = sensor;
						es_centro = true;
					} else {
						// Es una marca, decide de que lado es
						if (sensor < inicio_linea_principal) {
							m_izquierda = true;
						} else if (sensor > inicio_linea_principal) {
							m_derecha = true;
						}
					}
				}
				if (es_centro && fin_linea_principal == 0 && !sensores_detectando[sensor] && (inicio_linea_principal < sensor + 1)) {
					fin_linea_principal = sensor - 1;
				}
				if (sensor == (NUM_SENSORS - 1) && fin_linea_principal == 0) {
					fin_linea_principal = NUM_SENSORS - 1 ;
				}
				anterior = sensores_detectando[sensor];

			}
			// Si hay marca en alguno de los lados, y el lado con la marca ya la ha finalizado, comprobamos intersección.
			// // O, si hay marca en ambos lados y ambos lados han finalizado la marca, comprobamos interseccion

			if ((marcaDerecha != marcaIzquierda && ((marcaDerecha && marcaDerechaFin) || (marcaIzquierda && marcaIzquierdaFin)))
			        || ((marcaDerecha && marcaDerechaFin) && (marcaIzquierda && marcaIzquierdaFin))) {
				if (/*abs(inicio_linea_principal - fin_linea_principal) > 4 ||*/ num_lineas > 1) {
					led_state_time = 40;
					pasandoInterseccion = true;
					// Serial.print("\n");
				} else {
					led_state_time = 125;
				}
			} else {
				led_state_time = 250;
			}
		}
		// No está en else para corregir la primera vez que pasa a Intersección.
		if (pasandoInterseccion) {
			led_state_time = 40;
			num_lineas = 0;
			if ((marcaDerecha && !marcaIzquierda) || (marcaIzquierda && !marcaDerecha)) {
				if (marcaDerecha) {
					for (int sensor = (NUM_SENSORS - 1); sensor >= 0; sensor--) {
						// Serial.print(sensores_detectando[sensor]);
						// Serial.print("\t");

						if (anterior == false && sensores_detectando[sensor]) {
							num_lineas++;
							if (fin_linea_principal == 0) {
								inicio_linea_principal = sensor;
								es_centro = true;
							}
						} else if (anterior == true && es_centro && !sensores_detectando[sensor]) {
							fin_linea_principal = sensor + 1;
							es_centro = false;
						}

						anterior = sensores_detectando[sensor];
					}

					//Cambiar orden de inicio/fin por recorrer los sensores al reves
					int temp_inicio_linea = inicio_linea_principal;
					inicio_linea_principal = fin_linea_principal;
					fin_linea_principal = temp_inicio_linea;
				} else {
					for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
						// Serial.print(sensores_detectando[sensor]);
						// Serial.print("\t");
						if (anterior == false && sensores_detectando[sensor]) {
							num_lineas++;
							if (fin_linea_principal == 0) {
								inicio_linea_principal = sensor;
								es_centro = true;
							}
						} else if (anterior == true && es_centro && !sensores_detectando[sensor]) {
							fin_linea_principal = sensor - 1;
							es_centro = false;
						}

						anterior = sensores_detectando[sensor];
					}
				}
				if (num_lineas == 1) {
					pasandoInterseccion = false;
					marcaDerecha = false;
					marcaIzquierda = false;
					marcaDerechaFin = false;
					marcaIzquierdaFin = false;
				}

			} else if (marcaDerecha && marcaIzquierda) {
				// Buscar segunda lina por la derecha
				int num_lineas_derecha = 0;
				int num_lineas_izquierda = 0;
				int inicio_derecha = 0;
				int fin_derecha = 0;
				int inicio_izquierda = 0;
				int fin_izquierda = 0;
				for (int sensor = (NUM_SENSORS - 1); sensor >= 0; sensor--) {
					// Serial.print(sensores_detectando[sensor]);
					// Serial.print("\t");

					if (anterior == false && sensores_detectando[sensor]) {
						num_lineas_derecha++;
						if (num_lineas_derecha <= 2) {
							fin_derecha = sensor;
						}

					} else if (anterior == true && !sensores_detectando[sensor]) {
						if (num_lineas_derecha <= 2) {
							inicio_derecha = sensor + 1;
						}

					}

					anterior = sensores_detectando[sensor];
				}
				anterior = false;
				// Buscar segunda linea por la izquierda
				for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
					// Serial.print(sensores_detectando[sensor]);
					// Serial.print("\t");

					if (anterior == false && sensores_detectando[sensor]) {
						// inicio de linea
						num_lineas_izquierda++;
						if (num_lineas_izquierda <= 2) {
							inicio_izquierda = sensor;
						}

					} else if (anterior == true && !sensores_detectando[sensor]) {
						// fin de linea
						if (num_lineas_izquierda <= 2) {
							fin_izquierda = sensor - 1;
						}
					}

					anterior = sensores_detectando[sensor];
				}
				if (inicio_derecha == inicio_izquierda && fin_derecha == fin_izquierda) {
					inicio_linea_principal = inicio_izquierda;
					fin_linea_principal = fin_izquierda;
					num_lineas = num_lineas_izquierda;
				} else {
					if (abs(inicio_izquierda - ((NUM_SENSORS - 1) / 2)) < abs(inicio_derecha - ((NUM_SENSORS - 1) / 2))) {
						inicio_linea_principal = inicio_izquierda;
						fin_linea_principal = fin_izquierda;
						num_lineas = num_lineas_izquierda;
					} else {
						inicio_linea_principal = inicio_derecha;
						fin_linea_principal = fin_derecha;
						num_lineas = num_lineas_derecha;
					}
				}

				if (num_lineas == 1) {
					pasandoInterseccion = false;
					marcaDerecha = false;
					marcaIzquierda = false;
					marcaDerechaFin = false;
					marcaIzquierdaFin = false;
				}
			}

		}

		//Evitar desvío en angulos de 90º cuando pasa por el vértice en giros, no recto.
		if (marcaDerecha != marcaIzquierda && ((marcaDerecha && marcaDerechaFin) || (marcaIzquierda && marcaIzquierdaFin))) {
			// Serial.println(inicio_linea_principal - fin_linea_principal);
			if (abs(inicio_linea_principal - fin_linea_principal) > 2) {
				//Si el centro está entre ambos puntos, conservar central
				if (inicio_linea_principal <= ((NUM_SENSORS - 1) / 2) && fin_linea_principal >= ((NUM_SENSORS - 1) / 2)) {
					if (!marcaDerecha && marcaIzquierda) {
						inicio_linea_principal = ((NUM_SENSORS - 1) / 2) - 1;
						fin_linea_principal = ((NUM_SENSORS - 1) / 2);
					} else if (!marcaIzquierda && marcaDerecha) {
						inicio_linea_principal = ((NUM_SENSORS - 1) / 2);
						fin_linea_principal = ((NUM_SENSORS - 1) / 2) + 1;
					}
				}
			}
		}


		// Solo aplicar marcas cuando no se está en intersección.
		if (!pasandoInterseccion) {
			if (!marcaDerechaFin) {
				if (!marcaDerecha) {
					marcaDerecha = m_derecha;
				} else if (!m_derecha) {
					marcaDerechaFin = true;
				}
			}

			if (!marcaIzquierdaFin) {
				if (!marcaIzquierda) {
					marcaIzquierda = m_izquierda;
				} else if (!m_izquierda) {
					marcaIzquierdaFin = true;
				}
			}
		}


		// Serial.print("[");
		// Serial.print(num_lineas);
		// Serial.print("_");
		// Serial.print(inicio_linea_principal);
		// Serial.print("-");
		// Serial.print(fin_linea_principal);
		// Serial.print("|");
		// Serial.print(last_analog);
		// Serial.print("]\n");

		// for (int i = 0; i < NUM_SENSORS; i++) {
		// 	Serial.print(sensorValues[i]);
		// 	Serial.print("\t");
		// }
		// Serial.print(" inicial\n");
		for (int sensor = 0; sensor < inicio_linea_principal; sensor++) {
			sensorValues[sensor] = 0;
		}
		for (int sensor = fin_linea_principal + 1; sensor < NUM_SENSORS; sensor++) {
			sensorValues[sensor] = 0;
		}
		// for (int i = 0; i < NUM_SENSORS; i++) {
		// 	Serial.print(sensorValues[i]);
		// 	Serial.print("\t");
		// }
		// Serial.print(" editados\n");



		// Lógica de marcas
		if (millis() - lastStateLed > led_state_time) {
			stateLed = !stateLed;
			lastStateLed = millis();
		}
		if (!marcaDerechaFin) {
			if (marcaDerecha) {
				digitalWrite(LED_DERECHO, HIGH);
			} else {
				digitalWrite(LED_DERECHO, LOW);
			}
		} else if (marcaDerecha) {
			digitalWrite(LED_DERECHO, stateLed);
		}

		if (!marcaIzquierdaFin) {
			if (marcaIzquierda) {
				digitalWrite(LED_IZQUIERDO, HIGH);
			} else {
				digitalWrite(LED_IZQUIERDO, LOW);
			}
		} else if (marcaIzquierda) {
			digitalWrite(LED_IZQUIERDO, stateLed);
		}

		last_inicial = inicio_linea_principal;
	}
}


float calc_PID(float analogico) {
	float p = 0;
	double i = 0;
	float d = 0;
	float error = ideal - analogico;

	double in_kp = kp / 100;
	double in_kd = kd / 100;

	// Proporcional: constante · desviación
	p = (in_kp) * error;

	// Derivada: constante · derivada del error
	d = (in_kd) * ((error - lastError) / (millis() - lastMillis));
	lastMillis = millis();
	lastError = error;

	return p + i + d;
}

void calc_accel(short vel, bool obligado, bool recalcular, short time) {
	if (abs(vel - velReal) > 40 && vel != 0 && !obligado) {
		// Calcula la pendiente en caso de primera ejecucion de movimiento (si la pendiente de ambos motores es 0) o si se especifica forzado de calculo
		// TODO: Comprobar comportamiento cuando no es necesaria actualizacion => Cuando la vel objetivo es igual a la vel actual
		if (((m == 0) && vel != vel_ini) || recalcular) {
			// Si el robot ya estaba en movimiento obtiene sus velocidades reales, de las que parte para aceleración. Si no, usa el PWM_INICIAL.
			vel_ini = velReal;

			// Calcula la pendiente si la vel objetivo es distinta a la vel inicial (deberia solventar lo anterior de aceleraciones con vels iguales)

			m = (vel - (float)vel_ini) / (float)time;

			// Asigna a la vel real el valor inicial de aceleracion (para cuando empieza parado, darle velocidad inicial)
			// e indica que se esta cambiando_vel (acelerando o frenando) y guarda los millis del inicio de aceleracion
			cambiando_vel = true;
			millis_inicial_accel = millis();

		}

		// Asigna la velocidad en aceleracion correspondiente al tiempo transcurrido
		if (cambiando_vel) {
			// y   = 	mx    +    b
			// vel = m*millis + vel ini
			velReal = ((millis() - millis_inicial_accel) * m) + (float)vel_ini;

		}

		// Establece la velocidad en el máximo (objetivo) cuando pasa el tiempo de aceleracion, y deshabilita la asignación de velocidad
		if ((millis() - millis_inicial_accel) >= time) {
			cambiando_vel = false;
			velReal = vel;
			m = 0;
		}
	} else {
		velReal = vel;
	}
}

void console_debug(int section_debug) {
	if (!ENABLE_GENERAL_DEBUG)
		return;

	switch (section_debug) {
		case CALIBRATION_DEBUG:
			if (ENABLE_CALIBRATION_DEBUG) {
				Serial.println("Valores máximos:");
				for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
					Serial.print(maxVals[sensor]);
					((sensor + 1) != NUM_SENSORS) ? Serial.print("\t|\t") : Serial.println("\n");
				}
				Serial.println("Valores mínimos:");
				for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
					Serial.print(minVals[sensor]);
					((sensor + 1) != NUM_SENSORS) ? Serial.print("\t|\t") : Serial.println("\n");
				}
			}
			break;
		case READ_LINE_DEBUG:
			if (ENABLE_ANALOG_READ_DEBUG) {
				Serial.println("Lectura de sensores:");
				for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
					Serial.print(sensorValues[sensor]);
					((sensor + 1) != NUM_SENSORS) ? Serial.print("\t|\t") : Serial.println("\n");
				}
			}
			break;
	}
}

void console_debug(int section_debug, String msg, int paramA, int paramB) {
	if (!ENABLE_GENERAL_DEBUG)
		return;

	switch (section_debug) {
		case VEL_DEBUG:
			// do something
			break;
		case ANALOG_DEBUG:
			if (ENABLE_ANALOG_DEBUG) {
				Serial.println(msg);
				Serial.print(paramA);
				Serial.print("\t|\t");
				Serial.print(paramB);
				Serial.println("\n");
			}
			break;
	}
}

void set_speed(float correccion) {
	velI = velBase - correccion;
	velD = velBase + correccion;

	if (velReal > 50) {
		velI -= MOTOR_IZQUIERDO_OFFSET;
		velD -= MOTOR_DERECHO_OFFSET;
	}

	int pinD = MOTOR_DERECHO_ADELANTE;
	int pinI = MOTOR_IZQUIERDO_ADELANTE;
	if (velD > 255) {
		velD = 255;
		pinD = MOTOR_DERECHO_ADELANTE;
	} else if (velD < 0) {
		velD = abs(velD);
		if (velD > 255) {
			velD = 255;
		}
		pinD = MOTOR_DERECHO_ATRAS;
	}
	if (velI > 255) {
		velI = 255;
		pinI = MOTOR_IZQUIERDO_ADELANTE;
	} else if (velI < 0) {
		velI = abs(velI);
		if (velI > 255) {
			velI = 255;
		}
		pinI = MOTOR_IZQUIERDO_ATRAS;
	}

	if (!run) {
		run = true;
		digitalWrite(MOTOR_STBY, HIGH);
	}
	digitalWrite(MOTOR_DERECHO_ADELANTE, LOW);
	digitalWrite(MOTOR_DERECHO_ATRAS, LOW);
	digitalWrite(MOTOR_IZQUIERDO_ADELANTE, LOW);
	digitalWrite(MOTOR_IZQUIERDO_ATRAS, LOW);

	digitalWrite(pinD, HIGH);
	digitalWrite(pinI, HIGH);

	analogWrite(MOTOR_DERECHO_PWM   , velD);
	analogWrite(MOTOR_IZQUIERDO_PWM , velI);
	// Serial.println(velBase);
}
