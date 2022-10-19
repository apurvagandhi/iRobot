/*******************************************************************************
 * iRobotCommands.h, version 0.5
 *
 * Definitions for the low-level iRobot hardware.
 *
 * For high-level access to iRobot functions, see iRobot.h and iRobot.c.
 * You don't need to include this file if you include iRobot.h.
 *
 * Modified by kwalsh@cs.holycross.edu, based closely on the original oi.h
 *******************************************************************************/

#ifndef _IROBOTCOMMANDS_H_
#define _IROBOTCOMMANDS_H_

// Open Interface
// Command values
#define CmdStart        128
#define CmdBaud         129
#define CmdControl      130
#define CmdSafe         131
#define CmdFull         132
#define CmdSpot         134
#define CmdClean        135
#define CmdDemo         136
#define CmdDrive        137
#define CmdMotors       138
#define CmdLeds         139
#define CmdSong         140
#define CmdPlay         141
#define CmdSensors      142
#define CmdDock         143
#define CmdPWMMotors    144
#define CmdDriveWheels  145
#define CmdOutputs      147
#define CmdSensorStream 148
#define CmdSensorList   149
#define CmdSensorStreamControl 150
#define CmdIRChar       151
#define CmdWaitDist     156
#define CmdWaitAngle    157


// Sensor byte offsets 
#define SensorDataGroupPriority   0    // Group 0 : Safety + Telemetry + Power
#define SensorDataGroupAll        0    // Group 6 : Safety + Telemetry + Power + Enviro + Actuators
#define SensorDataGroupSafety     0    // Group 1 : Safety
#define SensorDataBumpDrop        0
#define SensorDataWall            1
#define SensorDataCliffL          2
#define SensorDataCliffFL         3
#define SensorDataCliffFR         4
#define SensorDataCliffR          5
#define SensorDataVWall           6
#define SensorDataOverC           7
// unused                         8
// unused                         9
#define SensorDataGroupTelemetry  10   // Group 2 : Telemetry
#define SensorDataIRChar          10
#define SensorDataButton          11
#define SensorDataDist            12
#define SensorDataDist0           13
#define SensorDataAng             14
#define SensorDataAng0            15
#define SensorDataGroupPower      16   // Group 3 : Power
#define SensorDataChargeState     16
#define SensorDataVolt            17
#define SensorDataVolt0           18
#define SensorDataCurr            19
#define SensorDataCurr0           20
#define SensorDataTemp            21
#define SensorDataCharge          22
#define SensorDataCharge0         23
#define SensorDataCap             24
#define SensorDataCap0            25
#define SensorDataGroupEnviro     26   // Group 4 : Enviro
#define SensorDataWallSig         26
#define SensorDataWallSig0        27
#define SensorDataCliffLSig       28
#define SensorDataCliffLSig0      29
#define SensorDataCliffFLSig      30
#define SensorDataCliffFLSig0     31
#define SensorDataCliffFRSig      32
#define SensorDataCliffFRSig0     33
#define SensorDataCliffRSig       34
#define SensorDataCliffRSig0      35
#define SensorDataInputs          36
#define SensorDataAInput          37
#define SensorDataAInput0         38
#define SensorDataChAvailable     39
#define SensorDataGroupActuators  40   // Group 5 : Actuators
#define SensorDataOIMode          40
#define SensorDataOISong          41
#define SensorDataOISongPlay      42
#define SensorDataStreamPckts     43
#define SensorDataVel             44
#define SensorDataVel0            45
#define SensorDataRad             46
#define SensorDataRad0            47
#define SensorDataVelR            48
#define SensorDataVelR0           49
#define SensorDataVelL            50
#define SensorDataVelL0           51


// Sensor packet sizes
#define SensorSizeGroupPriority   26    // Group 0 : Safety + Telemetry + Power
#define SensorSizeGroupAll        52    // Group 6 : Safety + Telemetry + Power + Enviro + Actuators
#define SensorSizeGroupSafety     10    // Group 1 : Safety
#define SensorSizeBumpDrop        1
#define SensorSizeWall            1
#define SensorSizeCliffL          1
#define SensorSizeCliffFL         1
#define SensorSizeCliffFR         1
#define SensorSizeCliffR          1
#define SensorSizeVWall           1
#define SensorSizeOverC           1
// unused                         1
// unused                         1
#define SensorSizeGroupTelemetry  6   // Group 2 : Telemetry
#define SensorSizeIRChar          1
#define SensorSizeButton          1
#define SensorSizeDist            2
#define SensorSizeAng             2
#define SensorSizeGroupPower      10   // Group 3 : Power
#define SensorSizeChargeState     1
#define SensorSizeVolt            2
#define SensorSizeCurr            2
#define SensorSizeTemp            1
#define SensorSizeCharge          2
#define SensorSizeCap             2
#define SensorSizeGroupEnviro     14   // Group 4 : Enviro
#define SensorSizeWallSig         2
#define SensorSizeCliffLSig       2
#define SensorSizeCliffFLSig      2
#define SensorSizeCliffFRSig      2
#define SensorSizeCliffRSig       2
#define SensorSizeInputs          1
#define SensorSizeAInput          2
#define SensorSizeChAvailable     1
#define SensorSizeGroupActuators  12   // Group 5 : Actuators
#define SensorSizeOIMode          1
#define SensorSizeOISong          1
#define SensorSizeOISongPlay      1
#define SensorSizeStreamPckts     1
#define SensorSizeVel             2
#define SensorSizeRad             2
#define SensorSizeVelR            2
#define SensorSizeVelL            2


// Sensor packet IDs
// Sensor packet sizes
#define SensorIDGroupPriority   0    // Group 0 : Safety + Telemetry + Power
#define SensorIDGroupAll        6    // Group 6 : Safety + Telemetry + Power + Enviro + Actuators
#define SensorIDGroupSafety     1    // Group 1 : Safety
#define SensorIDBumpDrop        7
#define SensorIDWall            8
#define SensorIDCliffL          9
#define SensorIDCliffFL         10
#define SensorIDCliffFR         11
#define SensorIDCliffR          12
#define SensorIDVWall           13
#define SensorIDOverC           14
// unused                       15
// unused                       16
#define SensorIDGroupTelemetry  2   // Group 2 : Telemetry
#define SensorIDIRChar          17
#define SensorIDButton          18
#define SensorIDDist            19
#define SensorIDAng             20
#define SensorIDGroupPower      3   // Group 3 : Power
#define SensorIDChargeState     21
#define SensorIDVolt            22
#define SensorIDCurr            23
#define SensorIDTemp            24
#define SensorIDCharge          25
#define SensorIDCap             26
#define SensorIDGroupEnviro     4   // Group 4 : Enviro
#define SensorIDWallSig         27
#define SensorIDCliffLSig       28
#define SensorIDCliffFLSig      29
#define SensorIDCliffFRSig      30
#define SensorIDCliffRSig       31
#define SensorIDInputs          32
#define SensorIDAInput          33
#define SensorIDChAvailable     34
#define SensorIDGroupActuators  5   // Group 5 : Actuators
#define SensorIDOIMode          35
#define SensorIDOISong          36
#define SensorIDOISongPlay      37
#define SensorIDStreamPckts     38
#define SensorIDVel             39
#define SensorIDRad             40
#define SensorIDVelR            41
#define SensorIDVelL            42


// Sensor bit masks
#define WheelDropFront  0x10
#define WheelDropLeft   0x08
#define WheelDropRight  0x04
#define WheelDropAny    0x1C
#define BumpLeft        0x02
#define BumpRight       0x01
#define BumpEither      0x03

#define ButtonAdvance   0x04
#define ButtonPlay      0x01

#define OverCurrentL    0x10
#define OverCurrentR    0x08
#define OverCurrentD3   0x04
#define OverCurrentD2   0x02
#define OverCurrentD1   0x01
#define OverCurrentAny  0x1F

// LED Bit Masks
#define LEDAdvance      0x08
#define LEDPlay         0x02
#define LEDsBoth        0x0A

// OI Modes
#define OIPassive       1
#define OISafe          2
#define OIFull          3


// Baud codes
#define Baud300         0
#define Baud600         1
#define Baud1200        2
#define Baud2400        3
#define Baud4800        4
#define Baud9600        5
#define Baud14400       6
#define Baud19200       7
#define Baud28800       8
#define Baud38400       9
#define Baud57600       10
#define Baud115200      11


// Drive radius special cases
#define RadStraight     32768
#define RadCCW          1
#define RadCW           -1



// Baud UBRRx values
#define Ubrr300         3839
#define Ubrr600         1919
#define Ubrr1200        959
#define Ubrr2400        479
#define Ubrr4800        239
#define Ubrr9600        119
#define Ubrr14400       79
#define Ubrr19200       59
#define Ubrr28800       39
#define Ubrr38400       29
#define Ubrr57600       19
#define Ubrr115200      9


// Command Module button and LEDs
#define UserButton        0x10
#define UserButtonPressed() (!(PIND & UserButton))

#define LED1              0x20
#define LED1Off()           (PORTD |= LED1)
#define LED1On()            (PORTD &= ~LED1)

#define LED2              0x40
#define LED2Off()           (PORTD |= LED2)
#define LED2On()            (PORTD &= ~LED2)

#define LEDBoth           0x60
#define LEDBothOff()        (PORTD |= LEDBoth)
#define LEDBothOn()         (PORTD &= ~LEDBoth)

// Create Port
#define RobotPwrToggle      0x80
#define RobotPwrToggleHigh() (PORTD |= 0x80)
#define RobotPwrToggleLow()  (PORTD &= ~0x80)

#define RobotPowerSense    0x20
#define RobotIsOn()        (PINB & RobotPowerSense)


// Command Module ePorts
#define LD2Over         0x04
#define LD0Over         0x02
#define LD1Over         0x01


#endif // _IROBOTCOMMANDS_H_
