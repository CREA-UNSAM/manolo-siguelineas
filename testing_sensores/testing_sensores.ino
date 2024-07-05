#include <Arduino.h>

//DEFINES
#define DEBUG 1

//PIN DEFINITIONS
const int PIN_LED = 14;  //Digital 8 | D8 | GPIO 14
const int PIN_BUTTON = 15;  //Digital 9 | D9 | GPIO 15

//MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 11;  //D5 | Digital 5| GPIO 11
const int PIN_MOTOR_R_1 = 12;  //D6 | Digital 6| GPIO 12
const int PIN_MOTOR_R_2 = 13;  //D7 | Digital 7| GPIO 13

//MOTOR LEFT
const int PIN_MOTOR_L_PWM = 5;   //D3 | Digital 3| GPIO 5
const int PIN_MOTOR_L_1 = 4;   //D2 | Digital 2| GPIO 4
const int PIN_MOTOR_L_2 = 6;   //D4 | Digital 4| GPIO 6

//SENSOR PINS
const int PIN_SENSOR_0 = 17;
const int PIN_SENSOR_1 = 23;
const int PIN_SENSOR_2 = 24;
const int PIN_SENSOR_3 = 25;
const int PIN_SENSOR_4 = 26;
const int PIN_SENSOR_5 = 27;
const int PIN_SENSOR_6 = 28;
const int PIN_SENSOR_7 = 18;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;


const int[CANT_ANALOG_SENSORS] PINS_ANALOG_SENSORS = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int[CANT_DIGITAL_SENSORS] PINS_DIGITAL_SENSORS = {PIN_SENSOR_0, PIN_SENSOR_7};


//GLOBAL CONSTANTS
#define ANALOG_SENSOR_THRESHOLD   1024
#define MOTORS_MAX_PWM_VALUE      1024


//GLOBAL VARIABLES
//TODO: no usar variables globales -> Analizar cual es la mejor practica...

int ledState = 0;

struct MotorsSpeeds {
  int leftSpeed;
  int rightSpeed;
};

struct SensorsData {
  int analogSensorValues[CANT_ANALOG_SENSORS];
  int digitalSensorValues[CANT_ALL_SENSORS];
};



void setup() {

  //initialize the serial communication
  Serial.begin(115200);
  Serial.println("STARTING THE PROGRAM");

  //initialize the LED pin as an output
  pinMode(PIN_LED, OUTPUT);

  //initialize the button pin as an input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  //initialize 6 analog inputs for sensors
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    pinMode(PINS_ANALOG_SENSORS[i], INPUT);
  }

  //initialize 2 digital inputs for sensors
  for (int i = 0; i < CANT_DIGITAL_SENSORS; i++) {
    pinMode(PINS_DIGITAL_SENSORS[i], INPUT);
  }

  //initialize the 3 outputs for each motors
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_1, OUTPUT);
  pinMode(PIN_MOTOR_L_2, OUTPUT);

  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_1, OUTPUT);
  pinMode(PIN_MOTOR_R_2, OUTPUT);

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

  SensorsData sensorData = readSensorsValues();

  if(DEBUG) {
    printSensorsValues(sensorData);
  } 
 
  MotorsSpeeds motorsSpeeds = calculateMotorsSpeeds(sensorData);

  if(DEBUG) {
    printMotorsSpeeds(motorsSpeeds);
  } 

  applySpeedsToMotors(motorsSpeeds);

  if(DEBUG){
    Serial.println("==============================================================");
  }

  delay(1000);
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

  SensorsData sensorData;
  
  //analog read
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorData.analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);
  }

  //digital read
  sensorData.digitalSensorValues[0] = digitalRead(PINS_DIGITAL_SENSORS[0]);

  //----analog to digital conversion
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorData.digitalSensorValues[i + 1] = sensorData.analogSensorValues[i] > (ANALOG_SENSOR_THRESHOLD / 2) ? 1 : 0;
  }

  sensorData.digitalSensorValues[CANT_ALL_SENSORS - 1] = digitalRead(PINS_DIGITAL_SENSORS[1]);

 
  //return the sensor data
  return sensorData;
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
  //calculate the motor speeds based on the sensor values
  int leftSpeed = 0;
  int rightSpeed = 0;

  //calculate the left motor speed
  leftSpeed = 255;

  //calculate the right motor speed
  rightSpeed = 255;

  //store the motor speeds in the motorSpeeds array
  motorSpeeds[0] = leftSpeed;
  motorSpeeds[1] = rightSpeed;
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
  if (motorSpeeds[0] > 0) {
    digitalWrite(motorLP1, HIGH);
    digitalWrite(motorLP2, LOW);
  } else {
    digitalWrite(motorLP1, LOW);
    digitalWrite(motorLP2, HIGH);
  }

  if (motorSpeeds[1] > 0) {
    digitalWrite(motorRP1, HIGH);
    digitalWrite(motorRP2, LOW);
  } else {
    digitalWrite(motorRP1, LOW);
    digitalWrite(motorRP2, HIGH);
  }

  analogWrite(motorLPWM, motorSpeeds[0]);
  analogWrite(motorRPWM, motorSpeeds[1]);
}