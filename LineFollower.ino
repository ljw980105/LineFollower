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
int leftSpeed = 100;

#pragma mark function prototypes

void listenForRemote();
void controlMotors();
void left();
void right();
void straight();

#pragma mark main arduino methods

void setup() {
    receiver.enableIRIn();
    Serial.begin(9600);

    pinMode(statusLED, OUTPUT);
    pinMode(leftMotor, OUTPUT);
    pinMode(leftForwardDirection, OUTPUT);
    pinMode(leftReverseDirection, OUTPUT);

}

void loop() {
    listenForRemote();
    if (state == 0) { // stop the robot
        digitalWrite(statusLED, LOW);

        analogWrite(leftMotor, 0);
        // set up the left motor
        digitalWrite(leftForwardDirection, HIGH);
        digitalWrite(leftReverseDirection, LOW);
    } else { // start the robot
        digitalWrite(statusLED, HIGH);

        //control the motors
        analogWrite(leftMotor, leftSpeed);
        // set up the left motor
        digitalWrite(leftForwardDirection, HIGH);
        digitalWrite(leftReverseDirection, LOW);
    }
}

#pragma mark main driving methods

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
            if (leftSpeed > 255) leftSpeed = 255;
        } else if (result == 0xFFE01F) { // decrease speed
            leftSpeed -= 10;
            if (leftSpeed < 0) leftSpeed = 0;
        }

        receiver.resume();
    }
}
