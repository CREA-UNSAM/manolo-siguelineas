#include <Arduino.h>
#include <PID_v1.h>

//DEFINES
#define DEBUG 1

//PIN DEFINITIONS
#define led       14  //Digital 8 | D8 | GPIO 14
#define button    15  //Digital 9 | D9 | GPIO 15

//MOTOR RIGHT
#define motorRPWM 11  //D5 | Digital 5| GPIO 11
#define motorRP1  12  //D6 | Digital 6| GPIO 12
#define motorRP2  13  //D7 | Digital 7| GPIO 13

//MOTOR LEFT
#define motorLPWM 5   //D3 | Digital 3| GPIO 5
#define motorLP1  4   //D2 | Digital 2| GPIO 4
#define motorLP2  6   //D4 | Digital 4| GPIO 6

//SENSOR PINS
//23 to 28
const int analogPins[]  = {23, 24, 25, 26, 27, 28};
const int digitalPins[] = {17, 18};

//GLOBAL CONSTANTS
#define ANALOG_SENSOR_THRESHOLD   1024
#define MAX_SPEED_MOTORS_PWM      1024


//GLOBAL VARIABLES
//TODO: no usar variables globales
int analogSensorValues[6] = {0, 0, 0, 0, 0, 0};
int digitalSensorValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int ledState = 0;
int motorSpeeds[2] = {0, 0};

// VARIABLES PID
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {

  //initialize the serial communication
  Serial.begin(115200);
  Serial.println("STARTING THE PROGRAM");

  //initialize the LED pin as an output
  pinMode(led, OUTPUT);

  //initialize the button pin as an input
  pinMode(button, INPUT_PULLUP);

  //initialize 6 analog inputs for sensors
  for (int i = 0; i < 6; i++) {
    pinMode(analogPins[i], INPUT);
  }

  //initialize 2 digital inputs for sensors
  for (int i = 0; i < 2; i++) {
    pinMode(digitalPins[i], INPUT);
  }

  //initialize 2 pwm outputs for motors
  int motorPins[] = {3, 4};
  for (int i = 0; i < 2; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  // Configurar el PID
  Setpoint = 0; // Ajustar el setpoint según sea necesario
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_SPEED_MOTORS_PWM, MAX_SPEED_MOTORS_PWM);

  //print the message to the serial monitor
  Serial.println("INITIALIZATION COMPLETED");


  //TODO: use the button to start the calibration
  // calibration();
  
}

void loop() {

  //when press the button, switch the state of the led
  if (digitalRead(button) == LOW) {
    digitalWrite(led, !digitalRead(led));
  }

  readSensorsValues();

  if(DEBUG) {
    printSensorsValues();
  } 
 
  calculateMotorsSpeeds();

  if(DEBUG) {
    printMotorsSpeeds();
  } 

  applySpeedsToMotors();

  if(DEBUG){
    Serial.println("==============================================================");
  }

  delay(100);
}

void calibration() {
  //print the message to the serial monitor
  Serial.println("CALIBRATION STARTED");

  //initialize the calibration values
  int calibrationValues[6] = {0, 0, 0, 0, 0, 0};

  //read the values of the sensors and store them in the calibrationValues array
  for (int i = 0; i < 6; i++) {
    calibrationValues[i] = analogRead(analogPins[i]);
  }

  //print the calibration values to the serial monitor
  for (int i = 0; i < 6; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(calibrationValues[i]);
  }

  //print the message to the serial monitor
  Serial.println("CALIBRATION COMPLETED");
}

void readSensorsValues() {
  //read the values of the sensors and store them in the analogSensorValues array
  for (int i = 0; i < 6; i++) {
    analogSensorValues[i] = analogRead(analogPins[i]);
  }

  //read the values of the sensors and store them in the digitalSensorValues array
  digitalSensorValues[0] = digitalRead(digitalPins[0]);
  for (int i = 0; i < 6; i++) {
    digitalSensorValues[i + 1] = analogSensorValues[i] > (ANALOG_SENSOR_THRESHOLD / 2) ? 1 : 0;
  }
  digitalSensorValues[7] = digitalRead(digitalPins[1]);
 
}

void printSensorsValues() {
  //print the values of the sensors to the serial monitor
  Serial.print("ANALOG  SENSOR VALUES: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(analogSensorValues[i]);
    Serial.print(" : ");
  }
  Serial.println("");

  Serial.print("DIGITAL SENSOR VALUES: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(digitalSensorValues[i] == 1 ? "[|]" : " ___");
  }
}

void calculateMotorsSpeeds() {
  // Calcular el valor de entrada del PID basado en los sensores

  //Sería necesario analizar cuál opción conviene más.

  // Método 1: Promedio de todos los sensores analógicos
  int sum = 0;
  for (int i = 0; i < 6; i++) {
    sum += analogSensorValues[i];
  }
  Input = sum / 6;

  // Método 2: Valor máximo de los sensores
  /*
  int maxVal = analogSensorValues[0];
  for (int i = 1; i < 6; i++) {
    if (analogSensorValues[i] > maxVal) {
      maxVal = analogSensorValues[i];
    }
  }
  Input = maxVal;
  */

  // Método 3: Peso diferenciado
  /*
  Input = (analogSensorValues[0] * 0.1) + (analogSensorValues[1] * 0.2) + 
          (analogSensorValues[2] * 0.3) + (analogSensorValues[3] * 0.3) + 
          (analogSensorValues[4] * 0.1);
  */
 //Método 4: Suma ponderada
 //Calcula pesos basados en la posición relativa al centro
  /*
  float weightedSum = 0.0;
  for (int i = 0; i < 6; i++) {
      float centerOffset = i - 2.5;  // Offset desde el centro
      float weight = centerOffset > 0 ? centerOffset + 0.5 : centerOffset - 0.5;  // Ajuste de peso gradual
      weightedSum += analogSensorValues[i] * weight;
  }
  */
  // Calcular el PID
  myPID.Compute();

  // Ajustar las velocidades de los motores basado en la salida del PID
  const int baseSpeed = 200; // Velocidad base de los motores
  //El output va a ir tomando valores positivos y negativos 
  motorSpeeds[0] = baseSpeed + Output; // Motor izquierdo
  motorSpeeds[1] = baseSpeed - Output; // Motor derecho

  // Asegurarse de que las velocidades no excedan los límites
  motorSpeeds[0] = constrain(motorSpeeds[0], -MAX_SPEED_MOTORS_PWM, MAX_SPEED_MOTORS_PWM);
  motorSpeeds[1] = constrain(motorSpeeds[1], -MAX_SPEED_MOTORS_PWM, MAX_SPEED_MOTORS_PWM);
}

void printMotorsSpeeds() {
  //print the motor speeds to the serial monitor
  Serial.println("");
  Serial.print("L = ");
  Serial.print(motorSpeeds[0]);

  Serial.print(" | R = ");
  Serial.println(motorSpeeds[1]);
}

void applySpeedsToMotors() {
  //apply the motor speeds to the motors

  // Motor izquierdo
  if (motorSpeeds[0] > 0) {
    digitalWrite(motorLP1, HIGH);
    digitalWrite(motorLP2, LOW);
    analogWrite(motorLPWM, motorSpeeds[0]);
  } else if (motorSpeeds[0] < 0) {
    digitalWrite(motorLP1, LOW);
    digitalWrite(motorLP2, HIGH);
    analogWrite(motorLPWM, -motorSpeeds[0]);//multiplica por -1 porque el valor es negativo
  } else {
    digitalWrite(motorLP1, LOW);
    digitalWrite(motorLP2, LOW);
    analogWrite(motorLPWM, 0);
  }

  // Motor derecho
  if (motorSpeeds[1] > 0) {
    digitalWrite(motorRP1, HIGH);
    digitalWrite(motorRP2, LOW);
    analogWrite(motorRPWM, motorSpeeds[1]);
  } else if (motorSpeeds[1] < 0) {
    digitalWrite(motorRP1, LOW);
    digitalWrite(motorRP2, HIGH);
    analogWrite(motorRPWM, -motorSpeeds[1]); 
  } else {
    digitalWrite(motorRP1, LOW);
    digitalWrite(motorRP2, LOW);
    analogWrite(motorRPWM, 0);
  }
}