//libraries import
#include <Adafruit_MotorShield.h>
#include <math.h>

//constants
const float WHEEL_RADIUS = 4.0;                  // Wheel radius in cm
const float COMPUTER_ACTUAL_SPEED_FACTOR = 0.1;  // Calibration factor
const float ROTATIONAL_RADIUS = 0.06;            // meters, = 1/2 * CAR_WIDTH
const float CAR_WIDTH = 0.12;                    // meters
const float CAR_LENGTH = 0.15;                   // meters
//const float PI = 3.14;

//define digital ports
const uint8_t rightLightSensorPin = 0;
const uint8_t middleLightSensorPin = 1;
const uint8_t leftLightSensorPin = 2;
const uint8_t magneticSensorPin = 3;



//define analog ports
const uint8_t distanceSensorPin = A0;


//define global motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//left motor connected to M1 port
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
//right motor connected to M2 port
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

//initialization 
void setupInputSensor(uint8_t pin){
  pinMode(pin, INPUT);
}

void setupOutputSensor(uint8_t pin){
  pinMode(pin, OUTPUT);
}

void detectMotor(){
  if (!AFMS.begin()){
    Serial.println("motor not found");
    while (1);
  }



}
//line following functions
bool lightSensorIsWhite(uint8_t pin, uint8_t T_s){
	//returns true if white pattern is detected underneath the light sensor
	//returns false if black pattern is detected underneath the light sensor
	bool val = (digitalRead(pin) == HIGH);
	delay(T_s);
	return val;
}

void keep_straight() {
  //car is on path so goes straight
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(300);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(300);

  if ((lightSensorIsWhite(leftLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)){
    align_left();
  }
  if ((lightSensorIsWhite(rightLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)) {
    align_right();
  }
}

void align_left() {
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(300);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  if (lightSensorIsWhite(middleLightSensorPin, 500) == 1) {
    delay(1000);
    keep_straight();
  }
}

void align_right() {
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(300);
  if (lightSensorIsWhite(middleLightSensorPin, 500) == 1) {
    delay(1000);
    keep_straight();
  }
}




//motor functions
float calculateRealSpeed(int speed){
  return speed * WHEEL_RADIUS * COMPUTER_ACTUAL_SPEED_FACTOR;
}

float calculateRunTime(float distance, float real_speed){
  if (real_speed == 0){
    return 0;
  } 
  else {
    return distance/real_speed;
  }
}

//main function
void setup() {
	Serial.begin(9600);

  setupInputSensor(rightLightSensorPin);
  setupInputSensor(middleLightSensorPin);
  setupInputSensor(leftLightSensorPin);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(lightSensorIsWhite(rightLightSensorPin, 500));
  delay(500);
}
