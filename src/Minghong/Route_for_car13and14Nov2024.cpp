// this file includes the routes that is exactly the same as the turtle, using already implmented functions
//libraries import
#include <Adafruit_MotorShield.h>
#include <math.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 25;  // Calibration factor
const float WEIGHT_FACTOR = 2; // the heavier, the less speed, weight factor < 0.
const float ROTATIONAL_RADIUS = 9.0*1.21;  // cm, = 1/2 * CAR_WIDTH
const float CAR_WIDTH = 18.0;          // cm
const float CAR_LENGTH = 19.0;         // cm
const float RIGHT_LEFT_TURN_DIFFERENCE = 7;
const float CLAW_LENGTH = 7;
// const float PI = 3.14;

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

void pick_up_rubbish() {
    Serial.println("Picking up rubbish.");
}

void drop_rubbish() {
    Serial.println("Dropping rubbish.");
}

// Define oneToGreen function
void oneToGreen() {
    keep_straight();
    forward_and_turn('R', 90);
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


void initializeMotors() {
    Serial.begin(9600);
    Serial.println("Initializing Motor Shield...");
    if (!AFMS.begin()) {
        Serial.println("Motor Shield not found. Check wiring.");
        while (1);  // Halt if Motor Shield is not found
    }
    Serial.println("Motor Shield initialized.");
}

float calculateRealSpeed(int speed) {
    float real_speed = speed / COMPUTER_ACTUAL_SPEED_RATIO * WEIGHT_FACTOR;
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
    float turnDistance = 3.14 * ROTATIONAL_RADIUS * (degrees / 180.0)*1.21;  // Calculate turn distance
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
    float turnDistance = 3.14 * ROTATIONAL_RADIUS * 2 * (degrees / 180.0)*1.3;  // Calculate turn distance
    Serial.println(turnDistance);
    unsigned long startTime = millis();                               // Start timing for the turn
    float time = calculateRunTime(turnDistance, calculateRealSpeed(200));  // Calculate time for turn


    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        leftMotor->run(FORWARD);
        // rightMotor->run(BACKWARD);
        leftMotor->setSpeed(200-RIGHT_LEFT_TURN_DIFFERENCE);
        rightMotor->setSpeed(0);
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        // leftMotor->run(BACKWARD);
        rightMotor->run(FORWARD);
        leftMotor->setSpeed(0);
        rightMotor->setSpeed(200);
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

void setup() {
    // Initialize motors and Serial communication
    initializeMotors();
}

void loop() {
    // Example of executing oneToGreen and oneToRed functions
    Serial.println("Executing oneToGreen route:");
    oneToGreen();
    
    delay(3000);  // Delay before the next action

    Serial.println("Executing oneToRed route:");
    oneToRed();

    delay(5000);  // Delay before repeating (or can end if only needed once)
}
