//libraries import
#include <Adafruit_MotorShield.h>
#include <math.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 25;  // Calibration factor
const float SPEED_FACTOR = 2.3; // the heavier, the less speed, weight factor < 0.
const float ROTATIONAL_RADIUS = 9.0*1.21;  // cm, = 1/2 * CAR_WIDTH
const float CAR_WIDTH = 20.0;          // cm
const float CAR_LENGTH = 15.0;         // cm
const float RIGHT_LEFT_TURN_DIFFERENCE = 7;
const float CLAW_LENGTH = 7;
// const float PI = 3.14;

//define digital ports
const uint8_t LightSensorPin1 = 2;
const uint8_t LightSensorPin2 = 3;
const uint8_t LightSensorPin3 = 4;
const uint8_t LightSensorPin4 = 5;
const uint8_t magneticSensorPin = 6;
const uint8_t buttonPin = 7;

#define echoPin                                            \
    2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin                                            \
    3 // attach pin D3 Arduino to pin Trig of HC-SR04   

// Motor Shield and Motor objects
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *leftMotor;
extern Adafruit_DCMotor *rightMotor;




// Motor Shield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Motor on port M2

void align_left();
void align_right();


float calculateRealSpeed(int speed) {
    float real_speed = speed / COMPUTER_ACTUAL_SPEED_RATIO * SPEED_FACTOR;
    Serial.print("Real Speed: ");
    Serial.println(real_speed);  // Print the calculated real speed
    return real_speed;
}

float calculateRunTime(float distance, float real_speed) {
    if (real_speed == 0) return 3000;
    float time = distance / real_speed;
    Serial.print("Calculated Time: ");
    Serial.println(time);  // Print the calculated time
    return time;
}

void runLeftMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);

    if (direction == 'F') {
        leftMotor->run(FORWARD);
    } else if (direction == 'B') {
        leftMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
        return;
    }
    Serial.print("distance: ");
    Serial.println(direction);

    leftMotor->setSpeed(speed);
    delay(time * 1000);
    leftMotor->run(RELEASE);
}

void runRightMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);

    if (direction == 'F') {
        rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        rightMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
        return;
    }
    Serial.print("distance: ");
    Serial.println(direction);

    rightMotor->setSpeed(speed);
    delay(time * 1000);
    rightMotor->run(RELEASE);
}

void runCar(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();  // Record the start time

    // Set the direction and speed for both motors
    if (direction == 'F') {
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        leftMotor->run(BACKWARD);
        rightMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
        return;
    }

    leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);
    Serial.print("Car is moving ");
    Serial.println(direction);


    // Run both motors for the calculated time
    while (millis() - startTime < time * 1000) {
        // Continue running both motors until the elapsed time reaches the target time
    }

    // Stop both motors after the time has elapsed
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void turnByMiddle(char direction, float degrees) {

    Serial.println("Car is turning!");
    // Step 2: Determine turn based on direction ('R' for right, 'L' for left)
    float turnDistance = 3.14 * ROTATIONAL_RADIUS * (degrees / 180.0);  // Calculate turn distance
    Serial.println(turnDistance);
    unsigned long startTime = millis();                               // Start timing for the turn
    float time = calculateRunTime(turnDistance, calculateRealSpeed(200));  // Calculate time for turn

    // Set both motor speeds
    leftMotor->setSpeed(200);
    rightMotor->setSpeed(200);

    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        leftMotor->run(FORWARD);
        rightMotor->run(BACKWARD);
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        leftMotor->run(BACKWARD);
        rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        // Backward turn: Left motor moves forward, Right motor moves backward (for 180-degree turn)
        leftMotor->run(FORWARD);
        rightMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'R' for right, 'L' for left, or 'B' for backward.");
        return;
    }

  

    // Run both motors for the calculated turn time
    while (millis() - startTime < time * 1000) {
        // Wait until the calculated turn time has passed
    }

    // Stop both motors after turning
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void turnByOneSide(char direction, float degrees) {

    Serial.println("Car is turning!");
    // Step 2: Determine turn based on direction ('R' for right, 'L' for left)
    float turnDistance = 3.14 * ROTATIONAL_RADIUS * 2 * (degrees / 180.0);  // Calculate turn distance
    Serial.println(turnDistance);
    unsigned long startTime = millis();                               // Start timing for the turn
    float time = calculateRunTime(turnDistance, calculateRealSpeed(200));  // Calculate time for turn


    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(0);
        leftMotor->setSpeed(200);
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(200);
        leftMotor->setSpeed(0);
    } else {
        Serial.println("Invalid direction. Use 'R' for right, 'L' for left, or 'B' for backward.");
        return;
    }

    // Run both motors for the calculated turn time
    while (millis() - startTime < time * 1000) {
        // Wait until the calculated turn time has passed
    }

    // Stop both motors after turning
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void forward_and_turn(char direction, float degree){
  runCar('F',230,CAR_LENGTH-CAR_WIDTH/2);
  turnByOneSide(direction,degree);
}

void forward_and_turn1(char direction, float degree){
  runCar('F',230,CAR_LENGTH);
  turnByMiddle(direction,degree);
}

void turn_around(){
  forward_and_turn('R',90);
  runCar('B',200,CAR_WIDTH);
  forward_and_turn('R',90);
}

void continue_straight() {
  runCar('F',250,10);
}

void stop_car() {
  runCar('F',0,12);
}

void drop_rubbish() {
  Serial.println('dropping rubbish');
}

// Define oneToGreen function
void oneToGreen() {
    keep_straight();
    forward_and_turn('R', 90);
    keep_straight();
    continue_straight();
    keep_straight();
    forward_and_turn('R', 90);
    keep_straight();
    forward_and_turn('R', 90);
    keep_straight();
    forward_and_turn('R', 90);
    
    runCar('F', 150, 30);  // Move forward to align for drop
    drop_rubbish();
    
    // Move backward to clear drop zone by CAR_WIDTH / 2
    runCar('B', 150, 30 + CAR_WIDTH / 2);
    turnByOneSide('L', 90);  // Turn left by one side
    keep_straight();
}

// Define oneToRed function
void oneToRed() {
    keep_straight();
    forward_and_turn('R', 90);
    keep_straight();
    continue_straight();
    keep_straight();
    forward_and_turn('R', 90);
    keep_straight();
    continue_straight();   
    keep_straight();
    forward_and_turn('R', 90);
    runCar('F', 150, 30);  // Move forward to align for drop
    drop_rubbish();
    runCar('B', 150, 30 + CAR_WIDTH / 2);
    turnByOneSide('R', 90);  // Turn right by one side
    runCar('F', 250, 70);    // Continue forward
    forward_and_turn('L', 90);
    keep_straight();
    continue_straight();   
    keep_straight();
}


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

bool button(){
	//returns true if white pattern is detected underneath the light sensor
	//returns false if black pattern is detected underneath the light sensor
	bool val = (digitalRead(buttonPin) == HIGH);
	return val;
}


int dist(){
  digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // wait for 2 ms to avoid
                          // collision in serial monitor

    digitalWrite(
        trigPin,
        HIGH); // turn on the Trigger to generate pulse
    delayMicroseconds(
        10); // keep the trigger "ON" for 10 ms to generate
             // pulse for 10 ms.

    digitalWrite(trigPin,
                 LOW); // Turn off the pulse trigger to stop
                       // pulse generation

    // If pulse reached the receiver echoPin
    // become high Then pulseIn() returns the
    // time taken by the pulse to reach the
    // receiver

    long duration = pulseIn(echoPin, HIGH);
    int dista = duration * 0.0344 / 2; // Expression to calculate
                                 // distance using time

    //Serial.print("Distance: ");
    //Serial.print(
        //dist); // Print the output in serial monitor
    //Serial.println(" cm");
    delay(100);
    return dista;
}

//main function
void setup() {
	Serial.begin(9600);
  Serial.println("setup");
  setupInputSensor(LightSensorPin1);
  setupInputSensor(LightSensorPin2);
  setupInputSensor(LightSensorPin3);
  setupInputSensor(LightSensorPin4);
	setupInputSensor(buttonPin);
  initializeMotors();
	pinMode(trigPin,
            OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

    // Serial Communication is starting with 9600 of
    // baudrate speed
    Serial.begin(9600);

    // The text to be printed in serial monitor
    Serial.println(
        "Distance measurement using Arduino Uno.");
    delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:

  keep_straight();
  forward_and_turn('R',90);
  keep_straight();
  // forward_and_turn('R',90);

}

void align_left() {
  Serial.println("align_left");
  while (lightSensorIsWhite(LightSensorPin3, 5) == 0) {
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(200);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(130);
    }
  keep_straight();
}
void align_right() {
  Serial.println("align_right");
  while (lightSensorIsWhite(LightSensorPin2, 5) == 0) {
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(130);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(200);
    }
  keep_straight();
}

void keep_straight() {
  Serial.println("keep straight");
  //car is on path so goes straight
  while((lightSensorIsWhite(LightSensorPin2, 0) == 1) && (lightSensorIsWhite(LightSensorPin3, 0) == 1) && (lightSensorIsWhite(LightSensorPin1, 0) == 0) && (lightSensorIsWhite(LightSensorPin4, 0) == 0)){
    
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(250);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(250);
  }

  if ((lightSensorIsWhite(LightSensorPin2, 5) == 1) && (lightSensorIsWhite(LightSensorPin3, 5) == 0) && (lightSensorIsWhite(LightSensorPin1, 0) == 0) && (lightSensorIsWhite(LightSensorPin4, 0) == 0)){
    align_left();
  }
  if ((lightSensorIsWhite(LightSensorPin3, 5) == 1) && (lightSensorIsWhite(LightSensorPin2, 5) == 0) && (lightSensorIsWhite(LightSensorPin1, 0) == 0) && (lightSensorIsWhite(LightSensorPin4, 0) == 0)) {
    align_right();
  }
  
}

void grab_rubbish() {

  Serial.println("go for grab");
  while (dist() >= 500) {
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(150);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(150);
  }
  servo1->run(FORWARD);
  servo1->setSpeed(150);
  delay(2000);
  servo1->setSpeed(0);
  
  while (button() == LOW) {
    servo2->run(FORWARD);
    servo2->setSpeed(150);
  }
  if (button() == HIGH){
    Serial.println("grabbed");
    servo2->run(FORWARD);
    servo2->setSpeed(0);
  }
  servo1->run(BACKWARD);
  servo1->setSpeed(150);
  delay(2000);
  servo1->setSpeed(0);
  Serial.println("lifted");
}


void drop_rubbish() {

  servo1->run(FORWARD);
  servo1->setSpeed(150);
  delay(2000);
  servo1->setSpeed(0);
  servo2->run(BACKWARD);
  servo2->setSpeed(150);
  delay(2000)
  servo2->setSpeed(0);
  servo1->run(BACKWARD);
  servo1->setSpeed(150);
  delay(2000);
  servo1->setSpeed(0);
}

