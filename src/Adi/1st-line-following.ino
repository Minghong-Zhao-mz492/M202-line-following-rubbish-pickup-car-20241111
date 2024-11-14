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
const uint8_t rightLightSensorPin = 2;
const uint8_t middleLightSensorPin = 3;
const uint8_t leftLightSensorPin = 4;
const uint8_t magneticSensorPin = 5;

// Motor Shield and Motor objects
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *leftMotor;
extern Adafruit_DCMotor *rightMotor;




// Motor Shield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Motor on port M2

void initializeMotors() {
    Serial.begin(9600);
    Serial.println("Initializing Motor Shield...");
    if (!AFMS.begin()) {
        Serial.println("Motor Shield not found. Check wiring.");
        while (1);  // Halt if Motor Shield is not found
    }
    Serial.println("Motor Shield initialized.");
}


//define analog ports
const uint8_t distanceSensorPin = A0;



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
  Serial.println("setup");
  setupInputSensor(rightLightSensorPin);
  setupInputSensor(middleLightSensorPin);
  setupInputSensor(leftLightSensorPin);
  initializeMotors();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("loop");
  Serial.println(lightSensorIsWhite(rightLightSensorPin, 500));
  if (lightSensorIsWhite(middleLightSensorPin, 500) == 1) {
    keep_straight();
  }
  if ((lightSensorIsWhite(leftLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)){
    align_left();
  }
  if ((lightSensorIsWhite(rightLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)) {
    align_right();
  }
  
}
void keep_straight() {
  //car is on path so goes straight
  Serial.println("keep straight");
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(250);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(250);

  if ((lightSensorIsWhite(leftLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)){
    align_left();
  }
  if ((lightSensorIsWhite(rightLightSensorPin, 500) == 1) && (lightSensorIsWhite(middleLightSensorPin, 500) == 0)) {
    align_right();
  }
}

void align_left() {
  while (lightSensorIsWhite(middleLightSensorPin, 500) == 0) {
    Serial.println("align_left");
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(150);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(100);
    }
}
void align_right() {
  while (lightSensorIsWhite(middleLightSensorPin, 500) == 0) {
    Serial.println("align_right");
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(100);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(150);
    }
}
