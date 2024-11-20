//libraries import
#include <Adafruit_MotorShield.h>
#include <math.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 14.5; 
const float SPEED_FACTOR = 1.18; // the heavier, the less speed, weight factor < 0.
const float CAR_WIDTH = 19 ;          // cm
const float ROTATIONAL_RADIUS = CAR_WIDTH/2;  // cm, = 1/2 * CAR_WIDTH
const float CAR_LENGTH = 15.5-1;         // cm
const float RIGHT_LEFT_TURN_DIFFERENCE = 1;
const float CLAW_LENGTH = 7;
// const float PI = 3.14;

//define digital ports
const uint8_t LightSensorPin1 = 2;
const uint8_t LightSensorPin2 = 3;
const uint8_t LightSensorPin3 = 4;
const uint8_t LightSensorPin4 = 5;
const uint8_t magneticSensorPin = 6;

// Motor Shield and Motor objects
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *leftMotor;
extern Adafruit_DCMotor *rightMotor;




// Motor Shield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);  // Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // Motor on port M2

void align_left();
void align_right();

void loop() {
  // put your main code here, to run repeatedly:
  runCar('F',200,20);
  // runLeftMotor('R',200,20);
  // keep_straight();
  forward_and_turn1('R',90);
  // turn('L',90);
  runCar('F',200,20);
  // runCar('F',200,110);
  // forward_and_turn('L',90);
  // runCar('F',200,20);
  // turn('L',90);
  // runCar('F',200,100);

  // keep_straight();
  // forward_and_turn('R',90);
  delay(3000);

}

float calculateRealSpeed(int speed) {
    float real_speed = speed / COMPUTER_ACTUAL_SPEED_RATIO * SPEED_FACTOR;
    Serial.print("Real Speed: ");
    Serial.println(real_speed);  // Print the calculated real speed
    return real_speed;
}

float calculateRunTime(float distance, float real_speed) {
    if (real_speed == 0) return 1000;
    float time = distance / real_speed;
    Serial.print("Calculated Time: ");
    Serial.println(time);  // Print the calculated time
    return time;
}

void runLeftMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();  // Record the start time

    // Set the direction and speed for both motors
    if (direction == 'F') {
        leftMotor->run(FORWARD);
        // rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        leftMotor->run(BACKWARD);
        // rightMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
        return;
    }

    leftMotor->setSpeed(speed);
    // rightMotor->setSpeed(speed);
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

void runRightMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();  // Record the start time

    // Set the direction and speed for both motors
    if (direction == 'F') {
        // leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        // leftMotor->run(BACKWARD);
        rightMotor->run(BACKWARD);
    } else {
        Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
        return;
    }

    // leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);
    Serial.print("Car is moving ");
    Serial.println(direction);


    // Run both motors for the calculated time
    while (millis() - startTime < time * 1000) {
        // Continue running both motors until the elapsed time reaches the target time
    }

    // Stop both motors after the time has elapsed
    // leftMotor->run(RELEASE);
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

void turn(char direction, float degrees) {

    Serial.println("Car is turning!");
    // Step 2: Determine turn based on direction ('R' for right, 'L' for left)
    float outerDistance = 3.14 * (CAR_LENGTH+ROTATIONAL_RADIUS) * (degrees / 180.0);  // Calculate turn distance
    float innerDistance = 3.14 * (CAR_LENGTH-ROTATIONAL_RADIUS) * (degrees / 180.0);  // Calculate turn distance
    float outerSpeed = 200.0;
    float innerSpeed = 200.0*innerDistance/outerDistance;
    unsigned long startTime = millis();                               // Start timing for the turn
    float time = calculateRunTime(outerDistance, calculateRealSpeed(outerSpeed));  // Calculate time for turn
    Serial.print(innerSpeed);

    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        // Set both motor speeds
        leftMotor->setSpeed(outerSpeed);
        rightMotor->setSpeed(innerSpeed);
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        // Set both motor speeds
        rightMotor->setSpeed(outerSpeed);
        leftMotor->setSpeed(innerSpeed);
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
    } else {
        Serial.println("Invalid direction. Use 'R' for right or 'L' for left.");
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
        time=time*1.05;
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        leftMotor->run(BACKWARD);
        rightMotor->run(FORWARD);
        time = time*1.15;
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
    float turnDistance = 3.14 * CAR_WIDTH * (degrees / 180.0)*1.1;  // Calculate turn distance
    Serial.println(turnDistance);
    unsigned long startTime = millis();                               // Start timing for the turn
    float time = calculateRunTime(turnDistance, calculateRealSpeed(200));  // Calculate time for turn


    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        runLeftMotor('F',200,turnDistance*1.15);
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        runRightMotor('F',200,turnDistance);
    } else {
        Serial.println("Invalid direction. Use 'R' for right, 'L' for left, or 'B' for backward.");
        return;
    }

    // Stop both motors after turning
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void forward_and_turn(char direction, float degree){
  runCar('F',200,CAR_LENGTH-CAR_WIDTH/2);
  turnByOneSide(direction,degree);
}

void forward_and_turn1(char direction, float degree){
  runCar('F',200,CAR_LENGTH);
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


//main function
void setup() {
	Serial.begin(9600);
  Serial.println("setup");
  setupInputSensor(LightSensorPin1);
  setupInputSensor(LightSensorPin2);
  setupInputSensor(LightSensorPin3);
  setupInputSensor(LightSensorPin4);
  initializeMotors();
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

