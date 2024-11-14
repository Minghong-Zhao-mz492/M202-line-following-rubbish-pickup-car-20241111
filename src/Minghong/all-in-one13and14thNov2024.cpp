
#include <Adafruit_MotorShield.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 25;  // Calibration factor
const float WEIGHT_FACTOR = 2; // the heavier, the less speed, weight factor < 0.
const float ROTATIONAL_RADIUS = 9.0*1.21;  // meters, = 1/2 * CAR_WIDTH
const float CAR_WIDTH = 18.0;          // meters
const float CAR_LENGTH = 19.0;         // meters
const float RIGHT_LEFT_TURN_DIFFERENCE = 7;
// const float PI = 3.14;

// Motor Shield and Motor objects
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *leftMotor;
extern Adafruit_DCMotor *rightMotor;

// Function declarations
void initializeMotors();
void runLeftMotor(char direction, int speed, float distance);
void runRightMotor(char direction, int speed, float distance);
float calculateRealSpeed(int speed);
float calculateRunTime(float distance, float real_speed);
void runCar(char direction, int speed, float distance);
void turn(char direction, float degrees);




// Motor Shield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);  // Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // Motor on port M2

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
    if (real_speed == 0) return 0;
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

void setup() {
    // Initialize anything you need here
  initializeMotors();
}

void loop() {
  // runCar('F', 500, 30); 
  // 700 on computer = 28 cm.
  // 400 on computer = 16 cm.
  // factor is 700/28 = 400/16 = 25
  runCar('F', 250, 20); 
  forward_and_turn('R',90.0);
  runCar('F', 250, 100-CAR_LENGTH); 
  forward_and_turn('L',90.0);
  runCar('F',250,95-CAR_LENGTH);
  forward_and_turn1('L',90);
  runCar('F', 250, 100-CAR_LENGTH); 
  delay(5000);
    // Main code loop (you can also put runLeftMotor here if it needs to run repeatedly)
}
