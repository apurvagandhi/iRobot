/*******************************************************************************
 * turnleft.c
 *
 * A simple iRobot program to test the turn accuracy.
 *******************************************************************************/
#define IROBOT_DEBUG
#include "iRobot.h"

void ShowAngle() {
    int16_t a = OdometerAngle();
    _ArduinoSendUInt(_OdometerReadings());
    if (a > 0)
        _ArduinoSend('L');
    else if (a < 0)
        _ArduinoSend('R');
    else
        _ArduinoSend('Z');
    _ArduinoSendInt(a);

    /*
    extern uint8_t _telemetry_angle_bucket[21]; // -10..10
    for (uint8_t i = 0; i <= 20; i++) {
        if (i == 10 || i == 11)
            _ArduinoSend('|');
        else
            _ArduinoSend(' ');
        _ArduinoSendInt(_telemetry_angle_bucket[i]);
        _telemetry_angle_bucket[i] = 0;
    }
    */
    _ArduinoSend('\n');
}

// For the instructor robot (7 of 9), using the script-rotate technique:
//  FAST 360 --> over-rotates by about 22 degrees
//  WALK 360 --> over-rotates by about 15 degrees
//  SLOW 360 --> over-rotates by about 10 degrees
//  FAST 180 --> over-rotates by about 15 degrees (and sensor is accurate after the turn)
//  WALK 180 --> over-rotates by about 10 degrees
//  SLOW 180 --> over-rotates by about 10 degrees
//  FAST  90 --> over-rotates by about 10 degrees (and sensor is accurate after the turn)
//  WALK  90 --> over-rotates by about 7-10 degrees (sensor is inaccurate by about half)
//  SLOW  90 --> over-rotates by about 7-10 degrees (and sensor is inaccurate mostly)

// For the instructor robot (7 of 9), using 15ms polling with +0.25 units compensation:
//  FAST 360 --> over-rotates by about 22 degrees
//  WALK 360 --> over-rotates by about 15 degrees
//  SLOW 360 --> over-rotates by about 10 degrees
//  FAST 180 --> over-rotates by about 15 degrees (and sensor is accurate after the turn)
//  WALK 180 --> over-rotates by about 10 degrees
//  SLOW 180 --> over-rotates by about 10 degrees
//  FAST  90 --> over-rotates by about 10 degrees (and sensor is accurate after the turn)
//  WALK  90 --> perfect
//  SLOW  90 --> over-rotates by about 7-10 degrees (and sensor is inaccurate mostly)

int main(void)
{
    WakeRobot();

    int16_t angle = 45;
    int16_t speed = SLOW ; // SLOW=100, WALK=200, FAST=500
        
    while (true) {
        SetModuleLEDs(true, false);
        WaitForBlackButton();
        //SetModuleLEDs(false, false);
        DriveAngle(speed, SPIN_LEFT, angle);
        ShowAngle();

        SetModuleLEDs(false, true);
        WaitForBlackButton();
        //SetModuleLEDs(false, false);
        DriveAngle(speed, SPIN_RIGHT, -angle);
        ShowAngle();
    }
}



