// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Velocista Para Pruebas.ino
    Created:	13/11/2019 17:44:09
    Author:     Daniel Burruchaga Sola (Buchspro)
*/

/********************************************************************************************************************************** * * *

  28 / 06 / 19------ - Ya lee bien los sensores y hace la posicion correctamente, tambien actua el pid pero hay que revisar las velocidades 
 ***************************************************************************************************************************************/
#define PWMA  PB9
#define AIN2  PB13
#define AIN1  PB12
#define BIN1  PB14
#define BIN2  PB15
#define PWMB  PA8

#define LimiteSensores  400
#define Numero_De_Muestras 50

#define Limite_errDif 80

#define Limite_Diferencia 150
#define LimiteVelocidad_mas_menos 120
#define Limite_dErr  80

#define Velocidad_Pos0 90
int Velocidad = 90;

bool ValorPA15 = LOW;
int Posicion = 0, contar = 0, last_Posicion = 0, diferenciaTiempo = 0, Suma_Valores = 0, Contador = 0, error = 0, lastErr = 0, Diferencia = 0, lastposition = 0, CountVel = 0, Diferencia_error = 0;
int SensorValue[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };
int SensorValueMap[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };

int SensorValue_BlancoMax[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };
int SensorValue_BlancoMin[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };

int SensorValue_NegroMax[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };
int SensorValue_NegroMin[11] = { 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0 };

unsigned long E_errDif_Tiempo = 0 , E_dErr = 0;
long dErr = 0;
long errDif_Tiempo = 0;
unsigned int cont_errDif_Tiempo = 0;
long Tiempo = 0;

bool Key_0 = 0, Key_25 = 0, Key_37 = 0, Key_50 = 0, Key_75 = 0, Key_100 = 0, Key_125 = 0, Key_150 = 0, Key_LastErr = 0, Key_Vel = 0, Key_DER = 0, Key_IZQ = 0;
double timeChange, lastMillisChange;
unsigned long lastTime, start_loop;
unsigned long now;
float kd = 6;     //1.2
float kp = 1.8;         //1
float ki = 0;       //1

long lastTiempo = 0, millis_vel = 0;
double lastMillisSerial = 0;

int Velocidad_Mas = 0, Velocidad_Menos = 0, lastVelocidad;

float Enviar[4] = { 0, 0, 0, 0 };
const int pinMotorA[3] = { PWMA, AIN2, AIN1 };
const int pinMotorB[3] = { PWMB, BIN1, BIN2 };

int buttonPushCounter = 0;
/*
  void Motors(int VelMot_A, int VelMot_B) {
  digitalWrite(PB5, HIGH);


  }*/

void Pulsador() {
	bool buttonState = digitalRead(PA15);
	bool lastButtonState = LOW;
	if (buttonState != lastButtonState) {
		if (buttonState == HIGH) {
			buttonPushCounter++;
			Serial.println("Detectada pulsación en: PA15");
		}
	}
	lastButtonState = buttonState;

}
void Calibrar() {
	char serialLect;
	while (buttonPushCounter <= 2) { //ESPERA AL PULSADOR

		ValorPA15 = digitalRead(PA15);
		bool AnteriorPA15 = LOW;
		if (Serial.available() > 0) serialLect = Serial.read();


		if (ValorPA15 != AnteriorPA15) {
			if (ValorPA15 == HIGH) {
				Serial.println(buttonPushCounter);
				buttonPushCounter++;
				delay(500);

				unsigned long last_millis_Calibrar = millis();

				if (buttonPushCounter == 1) { //ESPERA A QUE SEA PULSADO POR PRIMERA VEZ
					while ((millis() - last_millis_Calibrar) < 4000) { //Durante 3 segundos
						digitalWrite(PC13, HIGH);                       // Encender led PC13
						delay(1000);
						Serial.println("Calibrando color negro:");

						SensorValue[1] = analogRead(PA0);
						if (SensorValue[1] > SensorValue_NegroMax[1]) SensorValue_NegroMax[1] = SensorValue[1];
						if (SensorValue[1] < SensorValue_NegroMin[1]) SensorValue_NegroMin[1] = SensorValue[1];

						SensorValue[2] = analogRead(PA1);
						if (SensorValue[2] > SensorValue_NegroMax[2]) SensorValue_NegroMax[2] = SensorValue[2];
						if (SensorValue[2] < SensorValue_NegroMin[2]) SensorValue_NegroMin[2] = SensorValue[2];

						SensorValue[3] = analogRead(PA2);
						if (SensorValue[3] > SensorValue_NegroMax[3]) SensorValue_NegroMax[3] = SensorValue[3];
						if (SensorValue[3] < SensorValue_NegroMin[3]) SensorValue_NegroMin[3] = SensorValue[3];

						SensorValue[4] = analogRead(PA3);
						if (SensorValue[4] > SensorValue_NegroMax[4]) SensorValue_NegroMax[4] = SensorValue[4];
						if (SensorValue[4] < SensorValue_NegroMin[4]) SensorValue_NegroMin[4] = SensorValue[4];

						SensorValue[5] = analogRead(PA4);
						if (SensorValue[5] > SensorValue_NegroMax[5]) SensorValue_NegroMax[5] = SensorValue[5];
						if (SensorValue[5] < SensorValue_NegroMin[5]) SensorValue_NegroMin[5] = SensorValue[5];

						SensorValue[6] = analogRead(PA5);
						if (SensorValue[6] > SensorValue_NegroMax[6]) SensorValue_NegroMax[6] = SensorValue[6];
						if (SensorValue[6] < SensorValue_NegroMin[6]) SensorValue_NegroMin[6] = SensorValue[6];

						SensorValue[7] = analogRead(PA6);
						if (SensorValue[7] > SensorValue_NegroMax[7]) SensorValue_NegroMax[7] = SensorValue[7];
						if (SensorValue[7] < SensorValue_NegroMin[7]) SensorValue_NegroMin[7] = SensorValue[7];

						SensorValue[8] = analogRead(PA7);
						if (SensorValue[8] > SensorValue_NegroMax[8]) SensorValue_NegroMax[8] = SensorValue[8];
						if (SensorValue[8] < SensorValue_NegroMin[8]) SensorValue_NegroMin[8] = SensorValue[8];

						SensorValue[9] = analogRead(PB0);
						if (SensorValue[9] > SensorValue_NegroMax[9]) SensorValue_NegroMax[9] = SensorValue[9];
						if (SensorValue[9] < SensorValue_NegroMin[9]) SensorValue_NegroMin[9] = SensorValue[9];

						SensorValue[10] = analogRead(PB1);
						if (SensorValue[10] > SensorValue_NegroMax[10]) SensorValue_NegroMax[10] = SensorValue[10];
						if (SensorValue[10] < SensorValue_NegroMin[10]) SensorValue_NegroMin[10] = SensorValue[10];

						ValorPA15 = AnteriorPA15;
						SerialVoid();

					}
					digitalWrite(PC13, LOW);
				}
				if (buttonPushCounter == 2) { //ESPERA A QUE SEA PULSADO POR SEGUNDA VEZ
					while ((millis() - last_millis_Calibrar) < 4000) { //PRIMEROS 3 SEUGNDOS EN BLANCO
						digitalWrite(PC13, HIGH);  //ENCIENDE EL LED DURANTE LOS 3 SEGUNDOS DEL CALIBRADO
						delay(1000);
						Serial.println("Calibrando color blanco:");

						SensorValue[1] = analogRead(PA0);
						if (SensorValue[1] > SensorValue_BlancoMax[1]) SensorValue_BlancoMax[1] = SensorValue[1];
						if (SensorValue[1] < SensorValue_BlancoMin[1]) SensorValue_BlancoMin[1] = SensorValue[1];

						SensorValue[2] = analogRead(PA1);
						if (SensorValue[2] > SensorValue_BlancoMax[2]) SensorValue_BlancoMax[2] = SensorValue[2];
						if (SensorValue[2] < SensorValue_BlancoMin[2]) SensorValue_BlancoMin[2] = SensorValue[2];

						SensorValue[3] = analogRead(PA2);
						if (SensorValue[3] > SensorValue_BlancoMax[3]) SensorValue_BlancoMax[3] = SensorValue[3];
						if (SensorValue[3] < SensorValue_BlancoMin[3]) SensorValue_BlancoMin[3] = SensorValue[3];

						SensorValue[4] = analogRead(PA3);
						if (SensorValue[4] > SensorValue_BlancoMax[4]) SensorValue_BlancoMax[4] = SensorValue[4];
						if (SensorValue[4] < SensorValue_BlancoMin[4]) SensorValue_BlancoMin[4] = SensorValue[4];

						SensorValue[5] = analogRead(PA4);
						if (SensorValue[5] > SensorValue_BlancoMax[5]) SensorValue_BlancoMax[5] = SensorValue[5];
						if (SensorValue[5] < SensorValue_BlancoMin[5]) SensorValue_BlancoMin[5] = SensorValue[5];

						SensorValue[6] = analogRead(PA5);
						if (SensorValue[6] > SensorValue_BlancoMax[6]) SensorValue_BlancoMax[6] = SensorValue[6];
						if (SensorValue[6] < SensorValue_BlancoMin[6]) SensorValue_BlancoMin[6] = SensorValue[6];

						SensorValue[7] = analogRead(PA6);
						if (SensorValue[7] > SensorValue_BlancoMax[7]) SensorValue_BlancoMax[7] = SensorValue[7];
						if (SensorValue[7] < SensorValue_BlancoMin[7]) SensorValue_BlancoMin[7] = SensorValue[7];

						SensorValue[8] = analogRead(PA7);
						if (SensorValue[8] > SensorValue_BlancoMax[8]) SensorValue_BlancoMax[8] = SensorValue[8];
						if (SensorValue[8] < SensorValue_BlancoMin[8]) SensorValue_BlancoMin[8] = SensorValue[8];

						SensorValue[9] = analogRead(PB0);
						if (SensorValue[9] > SensorValue_BlancoMax[9]) SensorValue_BlancoMax[9] = SensorValue[9];
						if (SensorValue[9] < SensorValue_BlancoMin[9]) SensorValue_BlancoMin[9] = SensorValue[9];

						SensorValue[10] = analogRead(PB1);
						if (SensorValue[10] > SensorValue_BlancoMax[10]) SensorValue_BlancoMax[10] = SensorValue[10];
						if (SensorValue[10] < SensorValue_BlancoMin[10]) SensorValue_BlancoMin[10] = SensorValue[10];
						ValorPA15 = AnteriorPA15;
						SerialVoid();

					}
					digitalWrite(PC13, LOW);
				}
			}
		}
	}


}
void Posicion1() {

	SensorValue[1] = analogRead(PA0);
	SensorValue[2] = analogRead(PA1);
	SensorValue[3] = analogRead(PA2);
	SensorValue[4] = analogRead(PA3);
	SensorValue[5] = analogRead(PA4);
	SensorValue[6] = analogRead(PA5);
	SensorValue[7] = analogRead(PA6);
	SensorValue[8] = analogRead(PA7);
	SensorValue[9] = analogRead(PB0);
	SensorValue[10] = analogRead(PB1);

	SensorValueMap[1] = map(SensorValue[1], SensorValue_NegroMin[1], SensorValue_BlancoMax[1], 0, 1023); //PROBAR y luego partir en dos (Sensor > 125 = Blanco)  (Sensor < 125 = Negro)  si esto funciona solo hacer un sensado en vez de dos como ahora,( uno para blanco y otro para negro)  y hacer que gira (M1 100 M2 0) en la linea para coger el maximo y el minim
	SensorValueMap[2] = map(SensorValue[2], SensorValue_NegroMin[2], SensorValue_BlancoMax[2], 0, 1023);
	SensorValueMap[3] = map(SensorValue[3], SensorValue_NegroMin[3], SensorValue_BlancoMax[3], 0, 1023);
	SensorValueMap[4] = map(SensorValue[4], SensorValue_NegroMin[4], SensorValue_BlancoMax[4], 0, 1023);
	SensorValueMap[5] = map(SensorValue[5], SensorValue_NegroMin[5], SensorValue_BlancoMax[5], 0, 1023);
	SensorValueMap[6] = map(SensorValue[6], SensorValue_NegroMin[6], SensorValue_BlancoMax[6], 0, 1023);
	SensorValueMap[7] = map(SensorValue[7], SensorValue_NegroMin[7], SensorValue_BlancoMax[7], 0, 1023);
	SensorValueMap[8] = map(SensorValue[8], SensorValue_NegroMin[8], SensorValue_BlancoMax[8], 0, 1023);
	SensorValueMap[9] = map(SensorValue[9], SensorValue_NegroMin[9], SensorValue_BlancoMax[9], 0, 1023);
	SensorValueMap[10] = map(SensorValue[10], SensorValue_NegroMin[10], SensorValue_BlancoMax[10], 0, 1023);


	if (SensorValueMap[1] < LimiteSensores) {
		Suma_Valores = Suma_Valores + 113;
		Contador++;
	}
	if (SensorValueMap[2] < LimiteSensores) {
		Suma_Valores = Suma_Valores + 100;
		Contador++;
	}
	if (SensorValueMap[3] < LimiteSensores) {
		Suma_Valores = Suma_Valores + 75;
		Contador++;
	}
	if (SensorValueMap[4] < LimiteSensores) {
		Suma_Valores = Suma_Valores + 50;
		Contador++;
	}
	if (SensorValueMap[5] < LimiteSensores) {
		Suma_Valores = Suma_Valores + 25;
		Contador++;
	}
	if (SensorValueMap[6] < LimiteSensores) {
		Suma_Valores = Suma_Valores - 25;
		Contador++;
	}
	if (SensorValueMap[7] < LimiteSensores) {
		Suma_Valores = Suma_Valores - 50;
		Contador++;
	}
	if (SensorValueMap[8] < LimiteSensores) {
		Suma_Valores = Suma_Valores - 75;
		Contador++;
	}
	if (SensorValueMap[9] < LimiteSensores) {
		Suma_Valores = Suma_Valores - 100;
		Contador++;
	}
	if (SensorValueMap[10] < LimiteSensores) {
		Suma_Valores = Suma_Valores - 113;
		Contador++;
	}

	if ((Contador <= 2) && (Contador > 0)) {
		if (Suma_Valores == 0) {
			Posicion = 0;
		}
		else {
			Posicion = Suma_Valores / Contador;
		}

	}
	if ((Contador == 0) && (Posicion == 113)) Posicion = 125;
	if ((Contador == 0) && (Posicion == -113)) Posicion = -125; //Probar con 125

}
void Actuador() {
	//error = Posicion;
	// Si la posicion cambia guardar la diferencia de este cambio, y el tiempo que tarda en cambiar de posicion
	if (Posicion != lastErr) {
		Diferencia_error = Posicion - lastErr;
		timeChange = (millis() - lastMillisChange); //ver el valor de esta variable ESTA EN MILLIS y no se si poner en SEGUNDOS
	}
	lastMillisChange = millis();  // PROBANDO TIME CHANGE refresca lastMillisChange
	//===============================================================================================================================//
	E_errDif_Tiempo += (Diferencia_error * timeChange); //INTEGRAL PROBAR CON EL ERROR Y NO CON DIFERENCIA DE TIEMPO
	cont_errDif_Tiempo++;
	errDif_Tiempo = E_errDif_Tiempo / cont_errDif_Tiempo;
	if (cont_errDif_Tiempo == Numero_De_Muestras) {
		cont_errDif_Tiempo = 0;
		E_errDif_Tiempo = errDif_Tiempo;
	}
	if (errDif_Tiempo > Limite_errDif) errDif_Tiempo = Limite_errDif;
	if (errDif_Tiempo < (-Limite_errDif)) errDif_Tiempo = -Limite_errDif;

	//===============================================================================================================================//

	E_dErr += (Diferencia_error / timeChange);
	cont_errDif_Tiempo++;
	dErr = E_dErr / cont_errDif_Tiempo;
	if (cont_errDif_Tiempo == Numero_De_Muestras) {
		cont_errDif_Tiempo = 0;
		E_dErr = dErr;
	}
	if (dErr > Limite_errDif) dErr = Limite_dErr;
	if (dErr < (-Limite_errDif)) dErr = -Limite_errDif;

	//===============================================================================================================================//
	Diferencia = kp * Posicion + ki * errDif_Tiempo + kd * dErr;

	if (Diferencia > Limite_Diferencia) Diferencia = Limite_Diferencia;
	if ((Diferencia) < (-Limite_Diferencia)) Diferencia = (-1) * Limite_Diferencia;

	Velocidad_Mas = (Velocidad + Diferencia);


	if ((Velocidad_Mas) > LimiteVelocidad_mas_menos) Velocidad_Mas = LimiteVelocidad_mas_menos;    ///MIRAR BIEN
	if ((Velocidad_Mas) < (-LimiteVelocidad_mas_menos)) Velocidad_Mas = -LimiteVelocidad_mas_menos;    ///MIRAR BIEN
	// Velocidad_Mas = map(Velocidad_Mas, 0, 255, 0, 65535);  //CAMBIA LA RESOLUCION DE 8 BITS A 16


	Velocidad_Menos = (Velocidad - Diferencia);

	if (Velocidad_Menos > LimiteVelocidad_mas_menos) Velocidad_Menos = LimiteVelocidad_mas_menos;    ///MIRAR BIEN
	if (Velocidad_Menos < (-LimiteVelocidad_mas_menos)) Velocidad_Menos = -LimiteVelocidad_mas_menos;    ///MIRAR BIEN
	// Velocidad_Menos = map(Velocidad_Menos, 0, 255, 0, 25535);

	if (Posicion == 0) {
		/* int Last_Velocidad_Pos0 = Velocidad_Pos0;
		  while ((start_loop - millis() ) < (1000) ) {
		   if (Velocidad_Pos0 < Last_Velocidad_Pos0){
		   Velocidad_Pos0++;
		   }
		  }
		*/
		digitalWrite(PB5, HIGH);

		digitalWrite(PB12, HIGH);
		digitalWrite(PB13, LOW);
		//analogWrite(PB9, 255);
		analogWrite(PB9, 100);

		digitalWrite(PB14, LOW);
		digitalWrite(PB15, HIGH);
		//analogWrite(PA8, 255);
		analogWrite(PA8, 100);

	}
	else {
		digitalWrite(PB5, HIGH);

		if (Velocidad_Mas > 0) {
			digitalWrite(PB12, HIGH);
			digitalWrite(PB13, LOW);
			analogWrite(PB9, Velocidad_Mas);
		}

		if (Velocidad_Mas < 0) {
			digitalWrite(PB12, LOW);
			digitalWrite(PB13, HIGH);
			analogWrite(PB9, (Velocidad_Mas * (-1)));
		}

		if (Velocidad_Menos > 0) {
			digitalWrite(PB14, LOW); //PROBAR POLARIDAD
			digitalWrite(PB15, HIGH);
			analogWrite(PA8, Velocidad_Menos);
		}

		if (Velocidad_Menos < 0) {
			digitalWrite(PB14, HIGH);
			digitalWrite(PB15, LOW);
			analogWrite(PA8, (Velocidad_Menos * (-1)));
		}
	}
	/*

	  if (Posicion < 0) {
	  digitalWrite(PB5, HIGH);

	  digitalWrite(PB12, HIGH);
	  digitalWrite(PB13, LOW);
	  analogWrite(PB9, Velocidad_Menos);

	  digitalWrite(PB14, HIGH);
	  digitalWrite(PB15, LOW);
	  analogWrite(PA8, Velocidad_Mas);  //probar con pwmWrite

	  //Motors( Velocidad_Mas , Velocidad_Menos ); // ajustar dependiendo +  -

	  }
	  if (Posicion > 0) {
	  digitalWrite(PB5, HIGH);

	  digitalWrite(PB12, HIGH);
	  digitalWrite(PB13, LOW);
	  analogWrite(PB9, Velocidad_Mas);

	  digitalWrite(PB14, HIGH);
	  digitalWrite(PB15, LOW);
	  analogWrite(PA8, Velocidad_Menos);  //probar con pwmWrite

	  //Motors( Velocidad_Mas , Velocidad_Menos ); // ajustar dependiendo +  -

	  }*/
}
void Reset() {
	lastErr = Posicion;
	lastTiempo = Tiempo;
	Contador = 0;
	Suma_Valores = 0;
	last_Posicion = Posicion;
	lastTime = now;

}
void App() {
	char entrada;
	/* Serial.print("S1:  " );//
	  Serial.print(SensorValue[0] );
	  Serial.print("  S2:  " );
	  Serial.print(SensorValue[1]);
	  Serial.print("  S3:  " );
	  Serial.print(SensorValue[2]);
	  Serial.print("  S4:  " );
	  Serial.print(SensorValue[3]);
	  Serial.print("  S5:  " );
	  Serial.print(SensorValue[4]);
	  Serial.print("  S6:  ");
	  Serial.print(SensorValue[5]);
	  Serial.print("  count:  " );
	  Serial.print(Contador);
	  Serial.print("  Posicion:  " );
	  /* Serial.println(Posicion);
	  //Serial.print("  S6:  " );
	  //Serial.print(SensorValue5);*/
	  /*Serial.print("  Count:  " );
		Serial.print(Contador);
		Serial.print("       VelMot1:  " );
		Serial.print(  Velocidad_Mas);
		Serial.print("    VelMot2:  " );
		Serial.print(  Velocidad_Menos);
		Serial.print("       diferencia:  " );
		Serial.println(  Diferencia);
		Serial.print("       dErr:  " );
		Serial.print(  dErr);
		Serial.print("       kd:  " );
		Serial.print(  kd);
		Serial.print("       D:  " );
		Serial.print(  kd * dErr);
		Serial.print("       Diferencia_error:  " );
		Serial.print( Diferencia_error);
		Serial.print("   timeChange:  " );
		Serial.println(timeChange);
		//delay(100);*/

	if (Serial.available()) {
		char    inChar = Serial.read();

		if (inChar == 'a') {
			kd = kd + 0.1;
			sendAndroidValues();
		}
		if (inChar == 'b') {
			kd = kd - 0.1;
			sendAndroidValues();

		}
		if (inChar == 'c') {
			ki = ki - 0.1;
			sendAndroidValues();

		}
		if (inChar == 'd') {
			ki = ki + 0.1;
			sendAndroidValues();

		}
		if (inChar == 'e') {
			kp = kp - 0.1;
			sendAndroidValues();

		}
		if (inChar == 'f') {
			kp = kp + 0.1;
			sendAndroidValues();

		}
		if (inChar == 'g') {
			Velocidad = Velocidad - 2;
			sendAndroidValues();

		}
		if (inChar == 'h') {
			Velocidad = Velocidad + 2;
			sendAndroidValues();

		}
		if (inChar == 'i') {
			if ((Velocidad != lastVelocidad) && (Key_Vel = 0)) {
				lastVelocidad = Velocidad;
				Velocidad = 0;
				Key_Vel = 1;
			}
			sendAndroidValues();
		}
		if (inChar == 'j') {
			Velocidad = lastVelocidad;
			Key_Vel = 0;

			sendAndroidValues();

		}
	}
}
void sendAndroidValues() {
	Enviar[0] = (kd);
	Enviar[1] = (kp);
	Enviar[2] = (ki);
	Enviar[3] = (Velocidad);
	Serial.print('#');              //hay que poner # para el comienzo de los datos, así Android sabe que empieza el String de datos


  /*CAMBIAR POR IF Y UN CONTADOR */
	for (int k = 0; k < 4; k++)
	{
		Serial.print(Enviar[k]);
		if (k != 3) {
			Serial.print('+');            //separamos los datos con el +, así no es más fácil debuggear la información que enviamos
		}
	}
	Serial.println('~');               //con esto damos a conocer la finalización del String de datos
	delay(75);                       //IMPORTANTE CADA x ms envia si es muy bajo no recibe la app

	//sendAndroidValues();
}


void setup() {
	Serial.begin(57600); //COM16

	Serial.println("Bienvenido a BuchsPro:)");
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, LOW);

	pinMode(PA0, INPUT_ANALOG);
	pinMode(PA1, INPUT_ANALOG);
	pinMode(PA2, INPUT_ANALOG);
	pinMode(PA3, INPUT_ANALOG);
	pinMode(PA4, INPUT_ANALOG);
	pinMode(PA5, INPUT_ANALOG);
	pinMode(PA6, INPUT_ANALOG);
	pinMode(PA7, INPUT_ANALOG);
	pinMode(PB0, INPUT_ANALOG);
	pinMode(PB1, INPUT_ANALOG);

	Calibrar();

	pinMode(AIN2, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(PWMA, PWM);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
	pinMode(PWMB, PWM);
	pinMode(PB5, OUTPUT);
}
void SerialVoid() {
	if ((millis() - lastMillisSerial) > (1000))
		Serial.print(SensorValueMap[1]);
	Serial.print("   ");
	Serial.print(SensorValueMap[2]);
	Serial.print("   ");
	Serial.print(SensorValueMap[3]);
	Serial.print("   ");
	Serial.print(SensorValueMap[4]);
	Serial.print("   ");
	Serial.print(SensorValueMap[5]);
	Serial.print("   ");
	Serial.print(SensorValueMap[6]);
	Serial.print("   ");
	Serial.print(SensorValueMap[7]);
	Serial.print("   ");
	Serial.print(SensorValueMap[8]);
	Serial.print("   ");
	Serial.print(SensorValueMap[9]);
	Serial.print("   ");
	Serial.print(SensorValueMap[10]);
	/*Serial.print("   ");
	Serial.print(Posicion);
	Serial.print("   ");
	Serial.print(Contador);
	Serial.print("   ");
	Serial.print(Velocidad_Menos);
	Serial.print("   ");
	Serial.println(Velocidad_Mas);*/
	//delay(100);
	lastMillisSerial = millis();

}
void loop() {
	//now = millis();

	Posicion1();
	Actuador();
	SerialVoid();
	Reset();

}
