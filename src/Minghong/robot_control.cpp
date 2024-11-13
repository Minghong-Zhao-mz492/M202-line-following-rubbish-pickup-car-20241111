#include "robot_control.h"
#include <Adafruit_MotorShield.h>

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

float calculateRealSpeed(int speed) {
    return speed * WHEEL_RADIUS * COMPUTER_ACTUAL_SPEED_FACTOR;
}

float calculateRunTime(float distance, float real_speed) {
    if (real_speed == 0) return 0;
    return distance / real_speed;
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

    rightMotor->setSpeed(speed);
    delay(time * 1000);
    rightMotor->run(RELEASE);
}

void runCar(char direction, int speed, float distance) {
    runLeftMotor(direction, speed, distance);
    runRightMotor(direction, speed, distance);
}

void turn(char direction, float degrees) {
    // Step 1: Move the car forward a bit (optional, depending on your requirement)
    runCar('F', 300, CAR_LENGTH);
    
    // Stop the car momentarily
    runCar('F', 0, 1);

    // Step 2: Determine turn based on direction ('R' for right, 'L' for left)
    if (direction == 'R') {
        // Right turn: Left motor moves forward, Right motor moves backward
        runLeftMotor('F', 200, PI * ROTATIONAL_RADIUS * (degrees / 180.0));
        runRightMotor('B', 200, PI * ROTATIONAL_RADIUS * (degrees / 180.0));
    } else if (direction == 'L') {
        // Left turn: Right motor moves forward, Left motor moves backward
        runLeftMotor('B', 200, PI* ROTATIONAL_RADIUS * (degrees / 180.0));
        runRightMotor('F', 200, PI * ROTATIONAL_RADIUS * (degrees / 180.0));
    } else if (direction == 'B') {
        // Backward turn: Left motor moves forward, Right motor moves backward
    runLeftMotor('F', 200, PI * ROTATIONAL_RADIUS);
    runRightMotor('B', 200, PI * ROTATIONAL_RADIUS);
    }

    // Stop the car after turning
    runCar('F', 0, 1);
}


