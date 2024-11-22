// Libraries
#include <Adafruit_MotorShield.h>
#include <math.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 14.5; 
const float SPEED_FACTOR = 1.18; // Speed adjustment factor based on weight
const float CAR_WIDTH = 19; // cm
const float ROTATIONAL_RADIUS = CAR_WIDTH / 2; // Half car width
const float CAR_LENGTH = 15.5 - 1; // cm
const float RIGHT_LEFT_TURN_DIFFERENCE = 1;
const float CLAW_LENGTH = 7;

// Define digital ports
const uint8_t LightSensorPin1 = 2;
const uint8_t LightSensorPin2 = 3;
const uint8_t LightSensorPin3 = 4;
const uint8_t LightSensorPin4 = 5;
const uint8_t magneticSensorPin = 6;

// Define analog ports
const uint8_t distanceSensorPin = A0; // Analog pin for distance sensor

// Motor Shield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);  // Left motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // Right motor on port M1

// Function Declarations
void initializeMotors();
float calculateRealSpeed(int speed);
float calculateRunTime(float distance, float real_speed);
void runLeftMotor(char direction, int speed, float distance);
void runRightMotor(char direction, int speed, float distance);
void runCar(char direction, int speed, float distance);
void turn(char direction);
void turn1(char direction);
void claw_turn(char direction);
void keep_straight();
void align_left();
void align_right();
bool lightSensorIsWhite(uint8_t pin, uint8_t T_s);
void stop_car();
void continue_straight();
void pick_up_rubbish();
void drop_rubbish();
int magnetic_value();
char type_detection();
void setupInputSensor(uint8_t pin);
void setupOutputSensor(uint8_t pin);

// Route Functions
void fourToGreen();
void fourToRed();
void oneToGreen();
void oneToRed();
void twoToG();
void twoToR();
void threeToG();
void threeToR();
void route();

// **Setup Function**
void setup() {
    Serial.begin(9600);
    Serial.println("Setup...");
    setupInputSensor(LightSensorPin1);
    setupInputSensor(LightSensorPin2);
    setupInputSensor(LightSensorPin3);
    setupInputSensor(LightSensorPin4);
    initializeMotors();
}

// **Main Loop**
void loop() {
    keep_straight();
    turn('L');
    keep_straight();
    turn('R');
    delay(3000); // Pause for testing
}

// **Initialization Functions**
void setupInputSensor(uint8_t pin) {
    pinMode(pin, INPUT); // Configure pin as input
}

void setupOutputSensor(uint8_t pin) {
    pinMode(pin, OUTPUT); // Configure pin as output
}

void initializeMotors() {
    Serial.println("Initializing Motor Shield...");
    if (!AFMS.begin()) {
        Serial.println("Motor Shield not found. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield initialized.");
}

// **Line Following Functions**
bool lightSensorIsWhite(uint8_t pin, uint8_t T_s) {
    // Check if the sensor detects a white pattern (HIGH)
    bool val = (digitalRead(pin) == HIGH);
    delay(T_s); // Optional delay for sensor stability
    return val;
}

// **Speed and Timing Calculations**
float calculateRealSpeed(int speed) {
    return speed / COMPUTER_ACTUAL_SPEED_RATIO * SPEED_FACTOR;
}

float calculateRunTime(float distance, float real_speed) {
    if (real_speed == 0) return 1000; // Avoid divide by zero
    return distance / real_speed;
}

// **Motor Control Functions**
void runLeftMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();

    if (direction == 'F') leftMotor->run(FORWARD);
    else if (direction == 'B') leftMotor->run(BACKWARD);
    else return;

    leftMotor->setSpeed(speed);

    while (millis() - startTime < time * 1000) {}
    leftMotor->run(RELEASE);
}

void runRightMotor(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();

    if (direction == 'F') rightMotor->run(FORWARD);
    else if (direction == 'B') rightMotor->run(BACKWARD);
    else return;

    rightMotor->setSpeed(speed);

    while (millis() - startTime < time * 1000) {}
    rightMotor->run(RELEASE);
}

void runCar(char direction, int speed, float distance) {
    float real_speed = calculateRealSpeed(speed);
    float time = calculateRunTime(distance, real_speed);
    unsigned long startTime = millis();

    if (direction == 'F') {
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
    } else if (direction == 'B') {
        leftMotor->run(BACKWARD);
        rightMotor->run(BACKWARD);
    } else return;

    leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);

    while (millis() - startTime < time * 1000) {}
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

// **Turn Functions**
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
  runCar('F',200,CAR_LENGTH-CAR_WIDTH/2);
  turnByOneSide(direction,degree);
}

void forward_and_turn1(char direction, float degree){
  runCar('F',200,CAR_LENGTH);
  turnByMiddle(direction,degree);
}

void turn(char direction) {
    runCar('F', 200, CAR_LENGTH - ROTATIONAL_RADIUS);

    if (direction == 'L') {
	turnByOneSide('L',45);
        while (lightSensorIsWhite(LightSensorPin3, 5) == 0) {
            rightMotor->setSpeed(200);
            rightMotor->run(FORWARD);
        }
        rightMotor->run(RELEASE);
    } else if (direction == 'R') {
	turnByOneSide('R',45);
        while (lightSensorIsWhite(LightSensorPin2, 5) == 0) {
            leftMotor->setSpeed(200);
            leftMotor->run(FORWARD);
        }
        leftMotor->run(RELEASE);
    }
}

void turn1(char direction) {
    runCar('F', 200, CAR_LENGTH);

    if (direction == 'L') {
        turnByMiddle('L',45);
        while (!lightSensorIsWhite(LightSensorPin3, 5)) {
            leftMotor->setSpeed(200);
            rightMotor->setSpeed(200);
            leftMotor->run(BACKWARD);
            rightMotor->run(FORWARD);
        }
    } else if (direction == 'R') {
	turnByMiddle('R',45);
        while (!lightSensorIsWhite(LightSensorPin2, 5)) {
            leftMotor->setSpeed(200);
            rightMotor->setSpeed(200);
            leftMotor->run(FORWARD);
            rightMotor->run(BACKWARD);
        }
    } else {
        return;
    }
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void claw_turn(char direction){
	runCar('F',200,CLAW_LENGTH+CAR_LENGTH-CAR_WIDTH/2);

    if (direction == 'L') {
	    turnByOneSide('L',45);

        while (lightSensorIsWhite(LightSensorPin3, 5) == 0) {
            rightMotor->setSpeed(200);
            rightMotor->run(FORWARD);
        }
        rightMotor->run(RELEASE);
    } else if (direction == 'R') {
      turnByOneSide('R',45);
        while (lightSensorIsWhite(LightSensorPin2, 5) == 0) {
            leftMotor->setSpeed(200);
            leftMotor->run(FORWARD);
        }
        leftMotor->run(RELEASE);
    }
}

void align_left() {
    while (lightSensorIsWhite(LightSensorPin3, 5) == 0) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(200);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(130);
    }
    keep_straight();
}

void align_right() {
    while (lightSensorIsWhite(LightSensorPin2, 5) == 0) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(130);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(200);
    }
    keep_straight();
}

void keep_straight() {
    while ((lightSensorIsWhite(LightSensorPin2, 0) == 1) &&
           (lightSensorIsWhite(LightSensorPin3, 0) == 1) &&
           (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
           (lightSensorIsWhite(LightSensorPin4, 0) == 0)) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(250);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(250);
    }

    if ((lightSensorIsWhite(LightSensorPin2, 5) == 1) &&
        (lightSensorIsWhite(LightSensorPin3, 5) == 0)) align_left();

    if ((lightSensorIsWhite(LightSensorPin3, 5) == 1) &&
        (lightSensorIsWhite(LightSensorPin2, 5) == 0)) align_right();
}

// **Miscellaneous Functions**
void continue_straight() {
  runCar('F',250,10);
}

void stop_car() {
  runCar('F',0,12);
}

void pick_up_rubbish() {

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

int magnetic_value(){
  int value = (analogRead(magneticSensorPin));
  return value;
}

char type_detection() {
    if (magnetic_value()>10) return 'R';
    else return 'G';
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

float min_dist;

float scan_for_rubbish(char direction){
    // should add a turning function here
    min_dist = dist();
    for (i=0,i<100,i++){
        min_dist = min_dis > dist()? dist():min_dist;
        }
    return min_dist;
}

// Routes
void fourToGreen() {
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    runCar('F', 150, 30);  // Move forward to align for drop
    drop_rubbish();
    runCar('B', 150, 30 + CAR_WIDTH);
    turn('L');  // Turn left
    keep_straight();
}

void fourToRed() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();   
    keep_straight();
    turn('R');
    runCar('F', 150, 30);  // Move forward to align for drop
    drop_rubbish();
    runCar('B', 150, 30 + CAR_WIDTH);
    turn('R');  // Turn right
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();   
    keep_straight();
}

void oneToGreen() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();
    fourToGreen();
}

void oneToRed() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();
    fourToRed();
}

void twoToG() {
    keep_straight();
    turn('L');
    runCar('F', 150, 30);
    drop_rubbish();
    runCar('B', 150, 30 + CAR_WIDTH);
    turn('L');
    keep_straight();
}

void twoToR() {
    keep_straight();
    continue_straight();
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    runCar('F', 200, 30);
    drop_rubbish();
    runCar('B', 200, 30 + CAR_WIDTH);
    turn('R');
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
}

void threeToG() {
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
    turn('L');
    keep_straight();
    turn('R');
    runCar('F', 150, 30);
    drop_rubbish();
    runCar('B', 150, 30 + CAR_WIDTH);
    turn('L');
    keep_straight();
}

void threeToR() {
    keep_straight();
    turn('L');
    keep_straight();
    turn('L');
    runCar('F', 200, 30);
    drop_rubbish();
    runCar('B', 200, 30 + CAR_WIDTH);
    turn('R');
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
}

char rubbish_type;

void route() {
    keep_straight();
    pick_up_rubbish();
    rubbish_type = type_detection();

    claw_turn('L');

    if (rubbish_type == 'G') {
        oneToGreen();
    } else {
        oneToRed();
    }

    pick_up_rubbish();
    rubbish_type = type_detection();
    claw_turn('R');
    keep_straight();
    turn('R');

    if (rubbish_type == 'G') {
        fourToGreen();
    } else {
        fourToRed();
    }

    turn('L');
    keep_straight();
    pick_up_rubbish();
    rubbish_type = type_detection();
    keep_straight();
    turn('L');

    if (rubbish_type == 'G') {
        threeToG();
    } else {
        threeToR();
    }

    // This is for the rubbish off the line
    // turn('L');
    // runCar('F',200,60);
    // float distance = scan_for_rubbish('R');
    // runCar('F',200,distance);
    // pick_up_rubbish();
    // rubbish_type = type_detection();
    // runCar('B',200,distance+TURN_RADIUS);
    // turn('R');
    // keep_straight();
    // turn('R');

    // if (rubbish_type == 'G') {
    //     twoToG();
    // } else {
    //     twoToR();
    // }

    turn('L');
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    keep_straight();
    pick_up_rubbish();
    rubbish_type = type_detection();
    keep_straight();

    if (rubbish_type == 'G') {
        oneToGreen();
    } else {
        oneToRed();
    }

    turn('R');
    keep_straight();
    turn('L');
    runCar('F',200,50);
    // float distance = scan_for_rubbish('L');
    // runCar('F',200,distance);
    // pick_up_rubbish();
    // rubbish_type = type_detection();
    // runCar('B',200,distance+TURN_RADIUS);
    // turn('L');
    // keep_straight();
    // turn('R');

    // if (rubbish_type == 'G') {
    //     foutToG();
    // } else {
    //     fourToR();
    // }


    turn('L');
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    keep_straight();
    forward_and_turn('R',90);
    runCar('B',200,30);
}
