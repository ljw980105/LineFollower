# Line Follower

Arduino Code for the Line Follower Project of the IED class at RPI, Fall 2018.

## Notes

* Use a power supply module to power the motor.
* ~~The L293D has a chance of burning out.~~
* Pin 3 on the Arduino board is dead. Use pin 5 instead.
* Digital pins with the ~ symbol are the only pins capable of transmitting PWM signals.
* Power the IR receiver with Arduino instead of the power supply, because the motors be draw too much power.
* The back wheel of the car causes inconsistencies in directions. Preventing the yaw-like rotation of the wheel
will be an optimal solution.