#include <Arduino.h>
#include <IRremote.h>

/* HEX values for remote inputs:
 * play / pause button FF02FD
 * up button right of VOL-: FF906F
 * down button left of VOL-: FFE01F
 */

#define leftIR 5
#define rightIR 6
#define IRReceiver 7
#define statusLED 8
// left motor
#define leftMotor 2
#define leftForwardDirection 3
#define leftReverseDirection 4
// right motor
#define rightMotor 9
#define rightForwardDirection 10
#define rightReverseDirection 11

IRrecv receiver(IRReceiver);
decode_results results;

int state = 0;
uint8_t leftSpeed = 100;
uint8_t rightSpeed = 100;

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

//        /*
        analogWrite(leftMotor, leftSpeed);
        digitalWrite(leftForwardDirection, HIGH);
        digitalWrite(leftReverseDirection, LOW);
//        */
//        controlMotors();
    }
    delay(200);
}

#pragma markdriving methods

void controlMotors() {
    // TODO: May need to change to high
    int leftOnWhiteSurface = digitalRead(leftIR) == LOW;
    int rightOnWhiteSurface = digitalRead(rightIR) == LOW;

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

void left() {
    digitalWrite(leftMotor, 150);
    digitalWrite(rightMotor, 180);
    setDirections();
}

void right() {
    digitalWrite(leftMotor, 180);
    digitalWrite(rightMotor, 150);
    setDirections();
}

void straight() {
    // TODO: May need to use pid for more accurate results
    digitalWrite(leftMotor, leftSpeed);
    digitalWrite(rightMotor, rightSpeed);
    setDirections();
}

void stop () {
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
