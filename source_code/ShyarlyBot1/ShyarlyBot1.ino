
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
#define MOTOR_DERECHO_ADELANTE	    10
#define MOTOR_DERECHO_ATRAS		    9
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
float ideal = 0;
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
int minVal = 128; // Valor mínimo por debajo del cual se interpreta como línea
int maxVal = 129; // Valor máximo por encima del cual se interpreta como blanco
int minVals[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int maxVals[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



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
bool finGiro = false;
long last_interseccion = 0;
short tiempo_marca_interseccion = 100;


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
        CONFIGURACIONES COMPETICIÓN
  =========================================
*/
#define CONFIG_CIRCUITO_NORMAL 0
#define CONFIG_CIRCUITO_COMPLEJO 1

#define CONFIG_VELOCIDAD_NORMAL 0
#define CONFIG_VELOCIDAD_ALTA 1

#define CONFIG_kP 0
#define CONFIG_kI 1
#define CONFIG_kD 2
#define CONFIG_VEL 3
#define CONFIG_ACELERAR 4
#define CONFIG_AUMENTO_P 5
#define CONFIG_AUMENTO_D 6
#define CONFIG_MAX_AUMENTO 7
#define CONFIG_VEL_AUMENTO 8
#define CONFIG_MAX_ERROR 9

float configs[2][2][10];

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



PIDfromBT pid_calibrate(&kp, &ki, &kd, &velBase, &acelerar, &aumentoP, &aumentoD, &maxAumento, &velIncremento, &maxError, DEBUG);
void setup() {
	// Carga inicial de las configuraciones.
	load_configs();

	Serial.begin(9600);

	//	Declarar pines
	pins_init();
	if (!digitalRead(BTN_RACE)) {
		competicion = true;
		Serial.println("Competicion TRUE");
	}


	// Calibrando sensores
	sensor_calibrate();

}

void loop() {

	if (!competicion || (competicion && competicion_iniciada)) {
		if (!competicion) {
			pid_calibrate.update();
		}

		analogico = read_calibrated(sensorValues, analogico);
		float correccion = calc_PID(analogico);

		calc_accel(velBase, false, false, 250);
		set_speed(correccion);
		delay(1);
	} else if (competicion) {
		// Serial.println("NO COMP INIC");
		if (digitalRead(BTN_RACE)) {
			delay(100);
			if (!digitalRead(BTN_RACE)) {
				// Serial.println("SELECT CONF");
				select_config();
				while (!digitalRead(BTN_RACE)) {
					digitalWrite(LED_DERECHO, HIGH);
				}
				long millis_pre_start = millis();
				bool led_state = true;
				// long last_millis1 = 0;
				// long last_millis2 = 0;
				// short wait = 1000;
				while (millis() < (millis_pre_start + 5000)) {
					// if(((millis()-millis_pre_start)%350)<3 && millis()!=last_millis1 && wait >100){
					// 	wait -=45;
					// 	last_millis1 = millis();
					// 	// Serial.println("-wait");
					// }
					// if(((millis()-millis_pre_start)%wait)<3 && millis()!=last_millis2){
					// 	led_state = !led_state;
					// 	digitalWrite(LED_DERECHO, led_state? HIGH:LOW);
					// 	// Serial.println("LED TOGGLE");
					// 	last_millis2 = millis();
					// }
					if ((millis() - millis_pre_start) % 500 == 0) {
						led_state = !led_state;
						digitalWrite(LED_DERECHO, led_state);
					}
				}
				digitalWrite(LED_DERECHO, LOW);
				competicion_iniciada = true;
				// Serial.println("RUN!!");
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

int read_calibrated(short sensorValues[], int last_analog) {
	read_line(sensorValues);
	int lineSensors = 0;
	int sumVals = 0;
	int sumWeight = 0;
	double analog = 0;

	if (!digitalRead(SW_RASTREADOR)) {
		// Se establecen las marcas de giro detectadas
		if (((!marcaDerecha && !marcaIzquierdaFin) || (!marcaIzquierda && !marcaDerechaFin)) && sensorValues[7]<maxVal && (last_interseccion == 0 || ((millis()-last_interseccion) > tiempo_marca_interseccion))) {
			// Comprobar marca derecha
			if (!marcaIzquierdaFin && !marcaDerecha) {
				marcaDerecha = false;
				for (short sensor = 13; (sensor >= 10 && !marcaDerecha); sensor--) {
					marcaDerecha = (sensorValues[sensor] < maxVal) && !(sensorValues[sensor - 1] < maxVal);
				}
				marcaDerechaFin = false;
			}

			// Comprobar marca izquierda
			if (!marcaDerechaFin && !marcaIzquierda) {
				marcaIzquierda = false;
				for (short sensor = 1; (sensor <= 4 && !marcaIzquierda); sensor++) {
					marcaIzquierda = (sensorValues[sensor] < maxVal) && !(sensorValues[sensor + 1] < maxVal);
				}
				marcaIzquierdaFin = false;
			}

			if (marcaDerecha) {
				digitalWrite(LED_DERECHO, HIGH);
			}
			if (marcaIzquierda) {
				digitalWrite(LED_IZQUIERDO, HIGH);
			}
		}
		
			if ((!marcaIzquierdaFin && marcaIzquierda) || (!marcaDerechaFin && marcaDerecha)) {
				// Comprobar fin de marca derecha
				if (marcaDerecha) {
					marcaDerechaFin = true;
					for (short sensor = 10; sensor <= 13; sensor++) {
						// Si algun sensor detecta, no es fin de marca y sale del for
						if (sensorValues[sensor]<maxVal) {
							marcaDerechaFin = false;
							sensor = 13;
						}
					}
					// Si es fin de marca, ciega los sensores del lado opuesto si, en caso de tener marca, ya finalizó, o si no tiene marca
					if ((marcaDerechaFin && !marcaIzquierda) || (marcaDerechaFin && marcaIzquierda && marcaIzquierdaFin)) {
						cegarIzquierda = true;
					}
				}
				// Comprobar fin de marca izquierda
				if (marcaIzquierda) {
					marcaIzquierdaFin = true;
					for (short sensor = 1; sensor <= 4; sensor++) {
						// Si algun sensor detecta, no es fin de marca y sale del for
						if (sensorValues[sensor]<maxVal) {
							marcaIzquierdaFin = false;
							sensor = 4;
						}
					}
					// Si es fin de marca, ciega los sensores del lado opuesto si, en caso de tener marca, ya finalizó, o si no tiene marca
					if ((marcaIzquierdaFin && !marcaDerecha) || (marcaIzquierdaFin && marcaDerecha && marcaDerechaFin)) {
						cegarDerecha = true;
					}
				}
			}

			if (cegarDerecha) {
				if(pasandoInterseccion){
					finGiro = true;
				}
				for (short sensor = 10; sensor <= 14; sensor++) {
					// comprueba si alguno de los sensores detecta linea => está en la intersenccion
					if (!pasandoInterseccion && sensorValues[sensor] < maxVal) {
						pasandoInterseccion = true;
						sensor = 14;
					} else if (pasandoInterseccion) {
						if(sensorValues[sensor] < maxVal){
							finGiro = false;
							sensor = 14;
						}
					}
				}
				if (finGiro) {
					if(cegarIzquierda){
						digitalWrite(LED_DERECHO, LOW);
					}
					cegarDerecha = false;
					cegarIzquierda = false;
					marcaDerecha = false;
					marcaIzquierda = false;
					pasandoInterseccion =false;
					marcaDerechaFin = false;
					marcaIzquierdaFin = false;
					digitalWrite(LED_IZQUIERDO, LOW);
					finGiro = false;
					last_interseccion = millis();
				}
			}

			if (cegarIzquierda) {
				if(pasandoInterseccion){
					finGiro = true;
				}
				for (short sensor = 0; sensor <= 4; sensor++) {
					if (!pasandoInterseccion && sensorValues[sensor] < maxVal) {
						pasandoInterseccion = true;
						sensor = 4;
					} else if (pasandoInterseccion) {
						if(sensorValues[sensor] < maxVal){
							finGiro = false;
							sensor = 4;
						}
					}
				}
				if (finGiro) {
					if(cegarDerecha){
						digitalWrite(LED_IZQUIERDO, LOW);
					}
					cegarDerecha = false;
					cegarIzquierda = false;
					marcaDerecha = false;
					marcaIzquierda = false;
					pasandoInterseccion =false;
					marcaDerechaFin = false;
					marcaIzquierdaFin = false;
					digitalWrite(LED_DERECHO, LOW);
					finGiro = false;
					last_interseccion = millis();
				}
			}

		// Serial.print("cegarDerecha: ");
		// Serial.print(cegarDerecha);
		// Serial.print("\tcegarIzquierda: ");
		// Serial.println(cegarIzquierda);
			// Ciega los sensores
			if (cegarIzquierda || (marcaIzquierda && !marcaIzquierdaFin)) {
				for (short sensor = 0; sensor <= 4; sensor++) {
					sensorValues[sensor] = 255;
				}
			}
			if (cegarDerecha|| (marcaDerecha && !marcaDerechaFin)) {
				for (short sensor = 10; sensor <= 14; sensor++) {
					sensorValues[sensor] = 255;
				}
			}

			
	}

	for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
		sumVals += sensorValues[sensor] * sensorWeights[sensor];
		lineSensors += (sensorValues[sensor] < maxVal) ? 1 : 0;
		// Serial.print("(");
		// Serial.print(sensorValues[sensor]);
		// Serial.print("*");
		// Serial.print(sensorWeights[sensor]);
		// Serial.print(")+");
	}
	// Serial.print("/");
	// Serial.println(lineSensors);

	analog = (lineSensors > 0) ? (double)sumVals / (double)lineSensors : ((last_analog > ideal) ? (maxAnalog) : (-maxAnalog));
	// if((lineSensors>0 ||lineSensors == NUM_SENSORS)&&last_detected_line == 0){
	// 	last_detected_line = millis();
	// }
	// if((millis()-last_detected_line)>500){
	// 	velBase = 0;
	// 	kp = 0;
	// 	kd = 0;
	// 	ki = 0; 
	// }
	// Serial.println(analog);


	console_debug(ANALOG_DEBUG, "Analógico Parseado\tSensores Linea", analog, lineSensors);
	return map(analog, -maxAnalog, maxAnalog, -maxMappedAnalog, maxMappedAnalog);
}

void read_line(short sensorValues[]) {
	for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
		sensorValues[sensor] = map(muxAnalogRead(sensorPins[sensor]), minVals[sensor], maxVals[sensor], 0, 255);
		if (sensorValues[sensor] > maxVal) {
			sensorValues[sensor] = 255;
		} else if (sensorValues[sensor] < minVal) {
			sensorValues[sensor] = 0;
		}
		// sensorValues[sensor] = 255 - sensorValues[sensor];

	}
	console_debug(READ_LINE_DEBUG);
}

float calc_PID(float analogico) {
	float p = 0;
	double i = 0;
	float d = 0;
	float error = ideal - analogico;
	// sumError+=error;
		p = (kp) * error;
		d = (kd) * ((error - lastError) / (millis() - lastMillis));
	// i = (ki/100) * sumError;

	lastMillis = millis();
	lastError = error;

		if (digitalRead(SW_RASTREADOR))
			digitalWrite(LED_DERECHO, LOW);

	return p + i + d;
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
	velI = velReal + correccion;
	velD = velReal - correccion;

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

void load_configs() {

// ACC. OFF, CIRC. NORMAL, VEL. NORMAL
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kP]          = 0.7;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kD]          = 20;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_VEL]         = 180;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_ACELERAR]    = 0;

// ACC. OFF, CIRC. NORMAL, VEL. ALTA
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kP]          = 1.1;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kD]          = 40;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_VEL]         = 230;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_ACELERAR]    = 0;

// ACC. OFF, CIRC. COMPLEJO, VEL. NORMAL
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kP]          = 0.9;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kD]          = 30;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_VEL]         = 180;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_ACELERAR]    = 0;

// ACC. OFF, CIRC. COMPLEJO, VEL. ALTA
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kP]          = 1.2;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kD]          = 45;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_VEL]         = 220;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_ACELERAR]    = 0;

// ACC. ON, CIRC. NORMAL, VEL. NORMAL
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kP]          = 1;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_kD]          = 50;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_VEL]         = 180;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_NORMAL][CONFIG_ACELERAR]    = 1;

// ACC. ON, CIRC. NORMAL, VEL. ALTA
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kP]          = 1.1;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kI]          = 55;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_kD]          = 220;
	configs[CONFIG_CIRCUITO_NORMAL][CONFIG_VELOCIDAD_ALTA][CONFIG_VEL]         = 220;

// ACC. ON, CIRC. COMPLEJO, VEL. NORMAL
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kP]          = 1.2;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_kD]          = 50;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_NORMAL][CONFIG_VEL]         = 180;

// ACC. ON, CIRC. COMPLEJO, VEL. ALTA
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kP]          = 1.7;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kI]          = 0;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_kD]          = 80;
	configs[CONFIG_CIRCUITO_COMPLEJO][CONFIG_VELOCIDAD_ALTA][CONFIG_VEL]         = 120;

}

void select_config() {
	kp = 4;
	kd = 95;
	ki = 0;
	velBase = 150;
	// kp            = configs[!digitalRead(SW_CIR)][!digitalRead(SW_VEL)][CONFIG_kP];
	// ki            = configs[!digitalRead(SW_CIR)][!digitalRead(SW_VEL)][CONFIG_kI];
	// kd            = configs[!digitalRead(SW_CIR)][!digitalRead(SW_VEL)][CONFIG_kD];
	// velBase       = configs[!digitalRead(SW_CIR)][!digitalRead(SW_VEL)][CONFIG_VEL];
	// Serial.println(kp);
	// Serial.println(velBase);
}

