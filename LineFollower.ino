#include <Arduino.h>
#include <IRremote.h>

/* HEX values for remote inputs:
 * play / pause button FF02FD
 * up button right of VOL-: FF906F
 * down button left of VOL-: FFE01F
 * Important Note: Pin 3 is damaged!
 */

#define leftIR 7
#define rightIR 6
#define IRReceiver 8
#define statusLED 9
// left motor
#define leftMotor 10 // 10 is a pwm pin
#define leftForwardDirection 11
#define leftReverseDirection 12
// right motor
#define rightMotor 5 // pin 3 is damaged! - using pin 5 as pwm
#define rightForwardDirection 2
#define rightReverseDirection 4

IRrecv receiver(IRReceiver);
decode_results results;

int state = 0;

int leftTurnState = 0;
int rightTurnState = 0;

/** below speeds are the optimal ones to compensate for the build quality
 * between the left and right motors
 */
uint8_t leftSpeed = 130;
uint8_t rightSpeed = 200;

#pragma mark function prototypes

void listenForRemote();
void controlMotors();
void left();
void right();
void straight();
void stop();
void setDirections();

#pragma mark main arduino methods

void setup() {
    receiver.enableIRIn();
    Serial.begin(9600);

    pinMode(statusLED, OUTPUT);

    pinMode(leftMotor, OUTPUT);
    pinMode(leftForwardDirection, OUTPUT);
    pinMode(leftReverseDirection, OUTPUT);

    pinMode(rightMotor, OUTPUT);
    pinMode(rightForwardDirection, OUTPUT);
    pinMode(rightReverseDirection, OUTPUT);

    pinMode(leftIR, INPUT);
    pinMode(rightIR, INPUT);
}

void loop() {
    listenForRemote();
    if (state == 0) { // stop the robot
        digitalWrite(statusLED, LOW);
        stop();

    } else { // start the robot
        digitalWrite(statusLED, HIGH);

//        analogWrite(leftMotor, leftSpeed); // 140 - stable
//        digitalWrite(leftForwardDirection, HIGH);
//        digitalWrite(leftReverseDirection, LOW);
//
//        analogWrite(rightMotor, rightSpeed); // 160 - stable
//        digitalWrite(rightForwardDirection, HIGH);
//        digitalWrite(rightReverseDirection, LOW);

        controlMotors();
    }
}

#pragma mark driving methods

void controlMotors() {
    bool leftOnWhiteSurface = digitalRead(leftIR) == LOW;
    bool rightOnWhiteSurface = digitalRead(rightIR) == LOW;

    if ((leftOnWhiteSurface && rightOnWhiteSurface) || (!leftOnWhiteSurface && !rightOnWhiteSurface)) {
        straight();
    }
    if (leftOnWhiteSurface && !rightOnWhiteSurface) {
        right();
    }
    if (!leftOnWhiteSurface && rightOnWhiteSurface) {
        left();
    }
}

/**
 * Keep turning left until a black surface is reached
 */
void left() {
    while (digitalRead(rightIR) == LOW) { // if the right sensor is on the white surface
        analogWrite(leftMotor, 220); // 180
        analogWrite(rightMotor, 0); // 210
        setDirections();
        if (digitalRead(leftIR) == HIGH) leftTurnState = 1;
        if (digitalRead(leftIR) == LOW && leftTurnState == 1) {
            leftTurnState = 0;
            break;
        }
    }
}

/**
 * Keep turning right until a black surface is reached
 */
void right() {
    while (digitalRead(leftIR) == LOW) { // if the left sensor is on the white surface
        analogWrite(leftMotor, 0); // 210
        analogWrite(rightMotor, 220); // 180
        setDirections();
        if (digitalRead(rightIR) == HIGH) rightTurnState = 1;
        if (digitalRead(rightIR) == LOW && rightTurnState == 1) {
            rightTurnState = 0;
            break;
        }
    }
}

void straight() {
    // TODO: May need to use pid for more accurate results
    analogWrite(leftMotor, leftSpeed);
    analogWrite(rightMotor, rightSpeed);
    setDirections();
}

void stop() {
    analogWrite(leftMotor, 0);
    analogWrite(rightMotor, 0);
}

void setDirections()  {
    digitalWrite(leftForwardDirection, HIGH);
    digitalWrite(leftReverseDirection, LOW);
    digitalWrite(rightForwardDirection, HIGH);
    digitalWrite(rightReverseDirection, LOW);
}

#pragma mark helper methods

void listenForRemote() {
    if (receiver.decode(&results)) {
        long result = results.value;
        if (result == 0xFF02FD) { // pressed the play/pause button
            if (state == 0) { // change to driving state
                state = 1;
            } else if (state == 1) { // stop the robot
                state = 0;
            }
        } else if (result == 0xFF906F) { // increase speed
            leftSpeed += 10;
            rightSpeed += 10;
            if (leftSpeed > 255) leftSpeed = 255;
            if (rightSpeed > 255) rightSpeed = 255;
        } else if (result == 0xFFE01F) { // decrease speed
            leftSpeed -= 10;
            rightSpeed -= 10;
            if (leftSpeed < 0) leftSpeed = 0;
            if (rightSpeed < 0) rightSpeed = 0;
        }

        receiver.resume();
    }
}
