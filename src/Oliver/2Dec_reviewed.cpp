// Libraries
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <Servo.h>

// Constants
const float COMPUTER_ACTUAL_SPEED_RATIO = 14.5; 
const float SPEED_FACTOR = 1; // Speed adjustment factor based on weight
const float CAR_WIDTH = 19; // cm
const float ROTATIONAL_RADIUS = CAR_WIDTH / 2; // Half car width
const float CAR_LENGTH = 14.5; // cm
const float RIGHT_LEFT_TURN_DIFFERENCE = 1;
const float CLAW_LENGTH = 7;

// Define digital ports
const uint8_t LightSensorPin1 = 6;
const uint8_t LightSensorPin2 = 3;
const uint8_t LightSensorPin3 = 4;
const uint8_t LightSensorPin4 = 5;
const uint8_t blue_led = 8;
const uint8_t green_led = 9;
const uint8_t red_led = 10;
const uint8_t on_switch = 7;


//Define analog ports
int sensityPin = A0;
const uint8_t magneticSensorPin = A3;
const uint8_t magneticSensorPin2 = A1;


//constants for ultrasonic sensor
#define MAX_RANG (520)//the max measurement value of the module is 520cm
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

//Servo objects and there initial position
Servo myservo1;  
Servo myservo2;
int pos1 = 90;
int pos2;

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
    setupOutputSensor(blue_led);
    setupOutputSensor(green_led);
    setupOutputSensor(red_led);
    setupInputSensor(LightSensorPin1);
    setupInputSensor(LightSensorPin2);
    setupInputSensor(LightSensorPin3);
    setupInputSensor(LightSensorPin4);
  

    setupInputSensor(on_switch);
    setupInputSensor(sensityPin);
    initializeMotors();
    
    // The text to be printed in serial monitor
    Serial.println("Distance measurement using Arduino Uno.");
    delay(500);
    myservo1.attach(11);
    myservo2.attach(12);
}

// **Main Loop**
void loop() {

  
     while(digitalRead(on_switch)==HIGH){
       digitalWrite(blue_led,HIGH);
       delay(1000);
       digitalWrite(blue_led,LOW);
       delay(1000);
       myservo1.write(90);//170 up 90 down
       myservo2.write(90);
       //70 open, 110 closed
     }
    myservo1.write(170);//170 up 90 down
    myservo2.write(70);
    delay(1000);

    Serial.println(magnetic_value());
    route();
     // Pause for testing
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


float dist_t, sensity_t;

int dist(){
    sensity_t = analogRead(sensityPin);
// turn the ledPin on
    dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;//
    return dist_t;
}

int magnetic_value(){
  int value = (analogRead(magneticSensorPin) +analogRead(magneticSensorPin2));
  return value;
}

// **Speed and Timing Calculations**
float calculateRealSpeed(int speed) {
    return speed / COMPUTER_ACTUAL_SPEED_RATIO * SPEED_FACTOR;
}

float calculateRunTime(float distance, float real_speed) {
    if (real_speed == 0) return 1000; // Avoid divide by zero
    return distance / real_speed;
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

    leftMotor->setSpeed(speed-5);
    rightMotor->setSpeed(speed+10);

    while (millis() - startTime < time * 1000) {}
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
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

// NOT USED ANYMORE
void forward_and_turn(char direction, float degree){
  runCar('F',230,CAR_LENGTH-CAR_WIDTH/2);
  turnByOneSide(direction,degree);
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

void pure_turn(char direction) {

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
    while ((lightSensorIsWhite(LightSensorPin3, 5) == 0) &&
          (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
          (lightSensorIsWhite(LightSensorPin4, 0) == 0)) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(130);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(200);
    }
    keep_straight();
}

void align_right() {
    while ((lightSensorIsWhite(LightSensorPin2, 5) == 0) &&
          (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
          (lightSensorIsWhite(LightSensorPin4, 0) == 0)) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(200);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(130);
    }
    keep_straight();
}

void keep_straight() {
    while ((lightSensorIsWhite(LightSensorPin2, 0) == 1) &&
           (lightSensorIsWhite(LightSensorPin3, 0) == 1) &&
           (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
           (lightSensorIsWhite(LightSensorPin4, 0) == 0)
           && (dist()>=7) 
           ){
        digitalWrite(blue_led,HIGH);
        //delay(10);
        
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(250);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(250);
        //delay(490);
        digitalWrite(blue_led,LOW);
        //Serial.println("keep_straight");

    }

    if ((lightSensorIsWhite(LightSensorPin2, 0) == 1) &&
        (lightSensorIsWhite(LightSensorPin3, 0) == 0) &&
        (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
        (lightSensorIsWhite(LightSensorPin4, 0) == 0)
        && (dist()>=7) 
        ){
      align_left();
    }

    if ((lightSensorIsWhite(LightSensorPin3, 0) == 1) &&
        (lightSensorIsWhite(LightSensorPin2, 0) == 0) &&
        (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
           (lightSensorIsWhite(LightSensorPin4, 0) == 0)
         &&(dist()>=7) 
    ){
      align_right();
    }
    if ((lightSensorIsWhite(LightSensorPin3, 0) == 0) &&
        (lightSensorIsWhite(LightSensorPin2, 0) == 0) &&
        (lightSensorIsWhite(LightSensorPin1, 0) == 0) &&
           (lightSensorIsWhite(LightSensorPin4, 0) == 0) 
         &&(dist()>=10)
    ){
      keep_straight();
    }
    
    if ((dist()<=7)){
      grab_rubbish();
    }
      //leftMotor->run(RELEASE);
     //rightMotor->run(RELEASE);
}

char rubbish_type;
void grab_rubbish() {
    Serial.println("go for grab");
    while (dist() >= 7) {
        leftMotor->run(FORWARD);
        leftMotor->setSpeed(150);
        rightMotor->run(FORWARD);
        rightMotor->setSpeed(150);
    }
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    while (pos1>=90) {
        for (pos1 = 170; pos1 >= 90; pos1 -= 1) { // goes from 0 degrees to 180 degrees
            myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
            delay(50);                       // waits 15ms for the servo to reach the position
        }
     }
    while (pos2<=110) {
        for (pos2 = 70; pos2 <= 110; pos2 += 1) { // goes from 0 degrees to 180 degrees
            myservo2.write(pos2);              // tell servo to go to position in variable 'pos'
            delay(50);                       // waits 15ms for the servo to reach the position        }
        }
    }
    while (pos1<=170) {
        for (pos1 = 90; pos1 <= 170; pos1 += 1) { // goes from 0 degrees to 180 degrees
            myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
            delay(50);                       // waits 15ms for the servo to reach the position
        }
     }
    Serial.println("grabbed");
    digitalWrite(blue_led, HIGH);
    delay(2000);
  if (magnetic_value() >= 600) {
        digitalWrite(green_led,HIGH);
        delay(1000);
        digitalWrite(green_led,LOW);
        rubbish_type='G';
  }
  else {
        digitalWrite(red_led,HIGH);
        delay(1000);
        digitalWrite(red_led,LOW);
        rubbish_type='R';
  }

  Serial.println("lifted");
  delay(1000);
  keep_straight();
}

void drop_rubbish() {
    digitalWrite(blue_led,LOW);
    delay(500);
    myservo1.write(170);//170 up 90 down
    myservo2.write(70);//open
}

// **Miscellaneous Functions**
void continue_straight() {
    runCar('F',200,5);
}

void stop_car() {
    runCar('F',0,12);
}

void pick_up_rubbish() {
    Serial.println("Picking up rubbish.");
}


char type_detection() {
    // return 'G';
    return 'R';
}

// Routes
void fourToRed() {
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    runCar('F', 200, 10);  // Move forward to align for drop
    drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('L');  // Turn left
    keep_straight();
}

void fourToGreen() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();   
    keep_straight();
    turn('R');
    runCar('F', 200, 10);  // Move forward to align for drop
    // drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('R');  // Turn right
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();   
    keep_straight();
}

void oneToRed() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();
    fourToGreen();
}

void oneToGreen() {
    keep_straight();
    turn('R');
    keep_straight();
    continue_straight();
    fourToRed();
}

void twoToR() {
    keep_straight();
    turn('L');
    runCar('F', 200, 10);
    drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('L');
    keep_straight();
}

void twoToG() {
    keep_straight();
    continue_straight();
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    runCar('F', 200, 10);
    drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('R');
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
}

void threeToR() {
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
    turn('L');
    keep_straight();
    turn('R');
    runCar('F', 200, 10);
    drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('L');
    keep_straight();
}

void threeToG() {
    keep_straight();
    turn('L');
    keep_straight();
    turn('L');
    runCar('F', 200, 10);
    drop_rubbish();
    runCar('B', 200, 13 + CAR_WIDTH);
    pure_turn('R');
    keep_straight();
    turn('L');
    keep_straight();
    continue_straight();
    keep_straight();
}


void route() {
    runCar('F',200,10);
    keep_straight();

    turn('L');

    if (rubbish_type == 'G') {
        oneToGreen();
    } else {
        oneToRed();
    }

    pick_up_rubbish();
    turn('R');
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
    keep_straight();
    continue_straight();

    if (rubbish_type == 'G') {
        oneToGreen();
    } else {
        oneToRed();
    }

    turn('L');
    keep_straight();
    turn('R');
    keep_straight();
    turn('R');
    keep_straight();
    turn('L');
    keep_straight();
    turnByMiddle('R',200);
    runCar('B',200,2*CAR_LENGTH);
}
