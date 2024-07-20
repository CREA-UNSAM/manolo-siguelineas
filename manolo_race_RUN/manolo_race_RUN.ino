#include <Arduino.h>
#include <PID_v1.h>

//DEFINES
#define DEBUG 1

//PIN DEFINITIONS
const int PIN_LED = 8;           //D8 | Digital 8 | GPIO 14
const int PIN_BUTTON = 9;        //D9 | Digital 9 | GPIO 15 --???????? TODO: Revisar todos los pines

//MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 5;   //D5 | Digital 5 | GPIO 11
const int PIN_MOTOR_R_1 = 6;     //D6 | Digital 6 | GPIO 12
const int PIN_MOTOR_R_2 = 7;     //D7 | Digital 7 | GPIO 13

//MOTOR LEFT
const int PIN_MOTOR_L_PWM = 3;    //D3 | Digital 3 | GPIO 5
const int PIN_MOTOR_L_1 = 2;      //D2 | Digital 2 | GPIO 4
const int PIN_MOTOR_L_2 = 4;      //D4 | Digital 4 | GPIO 6

//SENSOR PINS
const int PIN_SENSOR_0 = 11;
const int PIN_SENSOR_1 = 14;
const int PIN_SENSOR_2 = 15;
const int PIN_SENSOR_3 = 16;
const int PIN_SENSOR_4 = 17;
const int PIN_SENSOR_5 = 18;
const int PIN_SENSOR_6 = 19;
const int PIN_SENSOR_7 = 12;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;


const int PINS_ANALOG_SENSORS[CANT_ANALOG_SENSORS] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int PINS_DIGITAL_SENSORS[CANT_DIGITAL_SENSORS] = {PIN_SENSOR_0, PIN_SENSOR_7};


//GLOBAL CONSTANTS
#define ANALOG_SENSOR_THRESHOLD   1024
#define ANALOG_SENSOR_MAX         1024
#define MOTORS_MAX_PWM_VALUE      1024

#define DELAY_MS_MAIN_LOOP  100

#define CANT_LONGPRESS_LEN    2500 / DELAY_MS_MAIN_LOOP  // 2.5 seconds
#define CANT_SHORTPRESS_LEN   500 / DELAY_MS_MAIN_LOOP   // 0.5 seconds
#define CANT_WAIT_TO_ACTION_LEN   200 / DELAY_MS_MAIN_LOOP   // 0.2 seconds

//STRUCTURES
struct MotorsSpeeds {
  int leftSpeed;
  int rightSpeed;
};

struct SensorsData {
  int analogSensorValues[CANT_ANALOG_SENSORS];
  int digitalSensorValues[CANT_ALL_SENSORS];
};

struct CalibrationValues {
  int maxValues[CANT_ANALOG_SENSORS];
  int minValues[CANT_ANALOG_SENSORS];
};

//ENUMS
enum events { EV_NONE = 0, EV_SHORTPRESS = 1, EV_LONGPRESS = 2};
enum state { STATE_SETUP= -1, STATE_STOP = 0, STATE_CALIBRATION = 1, STATE_RUNNING = 2};

//GLOBAL VARIABLES

int ledState = 0;
state currentState = STATE_SETUP;
CalibrationValues calibrationValues = CalibrationValues();

//VARIABLES PID
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
const int SPEED_BASE = 30; // Velocidad base de los motores

//PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//FUNCTIONS
void calibration();
SensorsData readSensorsValues();
void printSensorsValues(SensorsData sensorData);
MotorsSpeeds calculateMotorsSpeeds(SensorsData sensorData);
void printMotorsSpeeds(MotorsSpeeds motorSpeeds);
void applySpeedsToMotors(MotorsSpeeds motorSpeeds);
events handle_button();


void setup() {

  //initialize the serial communication
  Serial.begin(9600);
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

  // Configurar el PID
  Setpoint = 0; // Ajustar el setpoint según sea necesario
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MOTORS_MAX_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
  //print the message to the serial monitor
  Serial.println("INITIALIZATION COMPLETED");

  currentState = STATE_STOP;
}

void loop() {

  events event = handle_button();
  //print for serial monitor the event
  if(DEBUG){
    // Serial.print(event);
    Serial.print(digitalRead(PIN_BUTTON));
  }

  switch (currentState) {

    case STATE_STOP:
    {
      if (event == EV_SHORTPRESS) {
        currentState = STATE_RUNNING;
      } else if (event == EV_LONGPRESS) {
        currentState = STATE_CALIBRATION;
      }
      break;
    }

    case STATE_CALIBRATION:
    {
      calibration();
      currentState = STATE_STOP;
      break;
    }
    
    case STATE_RUNNING:
    {
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
      break;
    }
    
    default:
    {
      break;
    }
  
}


  delay(100);
}

void calibration() {
  //print the message to the serial monitor
  Serial.println("CALIBRATION STARTED");

  //blink the led to indicate the calibration process
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(50);
    digitalWrite(PIN_LED, LOW);
    delay(50);
  }

  //wait for 1 second and then start the calibration
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);


  //rotate the motors to calibrate the sensors
  // applySpeedsToMotors({-255, 255});

  //reset the calibration values
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    calibrationValues.maxValues[i] = 0;
    calibrationValues.minValues[i] = ANALOG_SENSOR_MAX;
  }


  //execute the reading for 3 second
  long startTime = millis();

  while (startTime + 3000 < millis()) {

    for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {

      int sensorValue = analogRead(PINS_ANALOG_SENSORS[i]);

      if (sensorValue > calibrationValues.maxValues[i]) {
        calibrationValues.maxValues[i] = sensorValue;
      }
      if (sensorValue < calibrationValues.minValues[i]) {
        calibrationValues.minValues[i] = sensorValue;
      }
    }
  }

  //print the calibration values to the serial monitor
  for (int i = 0; i < 6; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(calibrationValues.maxValues[i]);
    Serial.print(" | ");
    Serial.print(calibrationValues.minValues[i]);
    Serial.print(" # ");
  }

  // applySpeedsToMotors({-255, 255});
  

  //print the message to the serial monitor
  Serial.println("CALIBRATION COMPLETED");
}

SensorsData readSensorsValues() {

  SensorsData sensorData;
  
  //analog read
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorData.analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);
  }

  //digital read
  sensorData.digitalSensorValues[0] = digitalRead(PINS_DIGITAL_SENSORS[0]);

  //----analog to digital conversion
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    {
        sensorData.digitalSensorValues[i] = sensorData.analogSensorValues[i] > MOTORS_MAX_PWM_VALUE + MOTORS_MAX_PWM_VALUE / 2 ? 1 : 0;
        //map(sensorData.analogSensorValues[i], calibrationValues.minValues[i], calibrationValues.maxValues[i], 0, 100)
    }

  sensorData.digitalSensorValues[CANT_ALL_SENSORS - 1] = digitalRead(PINS_DIGITAL_SENSORS[1]);

  //return the sensor data
  return sensorData;
}

void printSensorsValues(SensorsData sensorData) {
  //print the values of the sensors to the serial monitor
  Serial.print("ANALOG  SENSOR VALUES: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorData.analogSensorValues[i]);
    Serial.print(" : ");
  }
  Serial.println("");

  Serial.print("DIGITAL SENSOR VALUES: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorData.digitalSensorValues[i] == 1 ? "[|]" : " ___");
  }
}

MotorsSpeeds calculateMotorsSpeeds(SensorsData sensorData) {

  MotorsSpeeds motorsSpeeds;
  // Calcular el valor de entrada del PID basado en los sensores

  if( sensorData.digitalSensorValues[0] == 1 && sensorData.digitalSensorValues[7] == 1){
    //Si ambos sensores digitales detectan una línea, se detiene el robot
    Input = 0;
    myPID.SetMode(MANUAL);
    myPID.SetOutputLimits(-MOTORS_MAX_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
    myPID.SetTunings(0, 0, 0);
    myPID.Compute();
    motorsSpeeds.leftSpeed = 0;
    motorsSpeeds.rightSpeed = 0;
    return motorsSpeeds;
  }

  //Sería necesario analizar cuál opción conviene más.

  // Método 1: Promedio de todos los sensores analógicos
  int sum = 0;
  for (int i = 0; i < 6; i++) {
    sum += sensorData.analogSensorValues[i];
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
  
  //int[6] weights = [-1, -4, -8, 8, 4, 1];
  
  //Input = 0;
  //for (int i = 0; i < 6; i++) {
   // Input += sensorData.analogSensorValues[i] * weights;
  //}
  
  
// 
//  Input = (analogSensorValues[0] * 0.1) + (analogSensorValues[1] * 0.2) + 
//          (analogSensorValues[2] * 0.3) + (analogSensorValues[3] * 0.3) + 
//          (analogSensorValues[4] * 0.1);
// 
 
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


  //El output va a ir tomando valores positivos y negativos 
  motorsSpeeds.leftSpeed = SPEED_BASE + Output; // Motor izquierdo
  motorsSpeeds.rightSpeed = SPEED_BASE - Output; // Motor derecho - EL MENOS ES A PROPOSITO NO BORRAR

  // Asegurarse de que las velocidades no excedan los límites
  motorsSpeeds.leftSpeed = constrain(motorsSpeeds.leftSpeed, -MOTORS_MAX_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
  motorsSpeeds.rightSpeed = constrain(motorsSpeeds.rightSpeed, -MOTORS_MAX_PWM_VALUE, MOTORS_MAX_PWM_VALUE);

  return motorsSpeeds;
}

void printMotorsSpeeds(MotorsSpeeds motorSpeeds) {
  //print the motor speeds to the serial monitor
  Serial.println("");
  Serial.print("L = ");
  Serial.print(motorSpeeds.leftSpeed);

  Serial.print(" | R = ");
  Serial.println(motorSpeeds.rightSpeed);
}

void applySpeedsToMotors(MotorsSpeeds motorSpeeds) {
  //apply the motor speeds to the motors

  // Motor izquierdo
  if (motorSpeeds.leftSpeed > 0) {
    digitalWrite(PIN_MOTOR_L_1, HIGH);
    digitalWrite(PIN_MOTOR_L_2, LOW);
    analogWrite(PIN_MOTOR_L_PWM, motorSpeeds.leftSpeed);
  } else if (motorSpeeds.leftSpeed < 0) {
    digitalWrite(PIN_MOTOR_L_1, LOW);
    digitalWrite(PIN_MOTOR_L_2, HIGH);
    analogWrite(PIN_MOTOR_L_PWM, -motorSpeeds.leftSpeed);//multiplica por -1 porque el valor es negativo
  } else {
    digitalWrite(PIN_MOTOR_L_1, LOW);
    digitalWrite(PIN_MOTOR_L_2, LOW);
    analogWrite(PIN_MOTOR_L_PWM, 0);
  }

  // Motor derecho
  if (motorSpeeds.rightSpeed > 0) {
    digitalWrite(PIN_MOTOR_R_1, HIGH);
    digitalWrite(PIN_MOTOR_R_2, LOW);
    analogWrite(PIN_MOTOR_R_PWM, motorSpeeds.rightSpeed);
  } else if (motorSpeeds.rightSpeed < 0) {
    digitalWrite(PIN_MOTOR_R_1, LOW);
    digitalWrite(PIN_MOTOR_R_2, HIGH);
    analogWrite(PIN_MOTOR_R_PWM, -motorSpeeds.rightSpeed); 
  } else {
    digitalWrite(PIN_MOTOR_R_1, LOW);
    digitalWrite(PIN_MOTOR_R_2, LOW);
    analogWrite(PIN_MOTOR_R_PWM, 0);
  }
}

events handle_button()
{
  static int button_pressed_counter = 0;
  static int button_not_pressed_counter = 0;
  events event = EV_NONE;
  int button_now_pressed = !digitalRead(PIN_BUTTON); // pin low -> pressed

  if (button_now_pressed){
    // Serial.println("Button pressed");
    ++button_pressed_counter;
    button_not_pressed_counter = 0;
  }    
  else{
    ++button_not_pressed_counter;
    // button_pressed_counter = 0;
  }

  if (button_not_pressed_counter >= CANT_SHORTPRESS_LEN){
    if (button_pressed_counter >= CANT_LONGPRESS_LEN){
    event = EV_LONGPRESS;
    }
    else if (button_pressed_counter >= CANT_SHORTPRESS_LEN){
      event = EV_SHORTPRESS;
    }
    button_pressed_counter = 0;
  }
  return event;
}
