/*******************************************************************************
 * iRobot.h, version 0.5
 *
 * Definitions for a simple high-level iRobot API.
 *
 * For low-level access to iRobot functions, see iRobotCommands.h and iRobot.c.
 * Include this file, and only this file, in iRobot program source code.
 *
 * Written by kwalsh@cs.holycross.edu, based closely on the original RobotAPI.c
 * See version history in iRobot.c
 *
 * Note: iRobot programs are written in the C language, not C++. Here is a
 * quick, 60-second primer on the differences you may see between iRobot
 * programming and regular c++ programing:
 *
 * (1) C++ constants look like this:
 *            const int METER = 1000;
 *     But C constants look like this:
 *            #define METER (1000)
 *
 * (2) There is no cout or cin.
 *
 * (3) There is no string type, and no reason to use double-quoted strings either.
 *
 * (4) Don't use float or double. All data should be integers.
 *
 * (5) There are some new types:
 *     int8_t   - Just like int, but ranges from -128 to +127.
 *     int16_t  - Just like int, but ranges from about -16000 to about +16000.
 *     int32_t  - Just like int, but ranges from about plus or minus two billion.
 *     uint8_t  - Just like unsigned int, but ranges from 0 to 255.
 *     uint16_t - Just like unsigned int, but ranges from 0 to about 32000.
 *     uint32_t - Just like unsigned int, but ranges from 0 to about four billion.
 *
 *******************************************************************************/

#ifndef _IROBOT_H_
#define _IROBOT_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "iRobotCommands.h"

/*
 * Useful constants.
 */

// Distances are measured in millimeters (mm). These constants can be used
// wherever a distance is called for. So instead of saying 3000, one can say
// 3*METER, and instead of saying 610, one can say 2*FOOT.
#define METER             (1000)   // 1000 mm = 1 meter
#define CENTIMETER        (10)     // 10 mm = 1 cm
#define FOOT              (305)    // 305 mm = 1 foot (approximate)
#define INCH              (25)     // 25 mm = 1 inch (approximate)

// Time is measured in milliseconds (ms). This constant can be used whenever a
// time or duration is called for. So instead of saying 2000, one can say
// 2*SECOND, for example.
#define SECOND            (1000)  // 1000 ms = 1 second

// Speed is measured in millimeters per second. These constants can be used
// wherever a velocity (i.e. speed) is called for. So instead of saying 500,
// which is the maximum iRobot velocity, one can say FAST, for example. You can
// use negative velocities, like -WALK, to go backwards.
#define FAST              (500)   // 500 mm per second (the maximum)
#define WALK              (200)   // 200 mm per second
#define SLOW              (100)   // 100 mm per second

// Turning radius is usually specified as a distance, so any of the above
// distance constants can be used. The largest allowed turning radius is 2
// meters, which would make the robot travel a circle 4 meter in diameter.
// Positive values mean turn to the left, negative means turn to the right.
// Larger values give a gentler curve, smaller values make a sharp turn. There
// are three special cases:
//   32768 is used to mean straight. It's like an infinitely large circle.
//   +1 and -1 are used to turn in place, either clockwise or counterclockwise.
#define STRAIGHT_AHEAD     (32768) // special value for radius to mean straight
#define GENTLE_LEFT        (2000)  // 2000 mm radius to the left (the maximum)
#define GENTLE_RIGHT       (-2000) // 2000 mm radius to the right (the maximum)
#define SHARP_LEFT         (200)   // 200 mm radius to the left
#define SHARP_RIGHT        (-200)  // 200 mm radius to the right
#define SPIN_LEFT          (1)     // special value for radius to mean spin left
#define SPIN_RIGHT         (-1)    // special value for radius to mean spin right

// Angles are measured in degrees. Positive means counterclockwise or left,
// negative means clockwise or right. There are no constants defined here,
// angles are usually obvious, eg. 90 for a quarter turn left, -180 for a half
// turn right, or 360 for a full revolution counterclockwise. To help you
// remember the signs, you can also write these as 90*SPIN_LEFT, -180*SPIN_RIGHT,
// an 360*SPIN_LEFT.

// One of the LEDs on the iRobot--the one near the power button--can do more
// than just On/Off. The color and intensity can be changed. These constants can
// be used for setting the color and intensity.
#define GREEN    (1)    // used for ledPColor parameter
#define ORANGE   (127)  // used for ledPColor parameter
#define RED      (255)  // used for ledPColor parameter
#define DIM      (63)   // used for ledPIntensity parameter
#define BRIGHT   (255)  // used for ledPIntensity parameter

/*
 * Powering up the iRobot.
 */

// Call this early on to initialize iRobot, e.g. in the first line of main().
// It will blink the lights a few times, make sure the wheels are stopped, and
// initialize some internal variables.
void WakeRobot();

/*
 * Time.
 */

// Get the time (in seconds) since last reboot. This function just
// returns UptimeMilliseconds()/1000.
uint16_t UptimeSeconds();

// Get the time (in milliseconds) since last reboot.
uint32_t UptimeMilliseconds();

// Get the current heartbeat number. There are roughly 4 heartbeats pre second, so this
// function returns a number between 0 and 3 and increments roughly each quarter
// second. The heartbeats are only approximately spaced in time.
uint8_t Heartbeat();

/*
 * Waiting and input.
 */

// Wait for time_ms milliseconds to elapse.
void Delay(uint16_t time_ms);

// Start a timer countdown from from time_ms milliseconds. You can call
// IsTimerExpired() to find out whether the timer has reached zero or not. The
// timer is only accurate to within +/- 10 ms precision, at best.
void StartTimer(uint16_t time_ms);

// Find out whether the timer has expired.
bool IsTimerExpired();

// Wait at most time_ms milliseconds for the iRobot to bump something.
// Returns true if bumped in the specified time frame, false otherwise.
// The iRobot also backs up about 1 cm if something was bumped.
bool WaitForBump(uint16_t time_ms);

// Wait for the black button on the command module to be pressed and released.
// Returns the number of milliseconds the button was held. 
uint16_t WaitForBlackButton();

// Wait up to 2 seconds for the black button on the command module to be
// pressed. Returns true if the button was pressed within that time, false
// otherwise. This allows for a very simple but tedious kind of user input. 
bool GetBitFromUser();

// Wait for the black button on the command module to be pressed. Returns a
// number from 0 to 15 depending on how long the button was held down. At first,
// the LEDs are turned off, and a counter is set to 0. Each second that the
// button is held down, the counter is incremented by one, and displayNum is
// called to light up the LEDs to show the current counter value. This allows
// for a sligthly more sophisticated kind of user input with some feedback
// provided using the LEDs.
uint8_t GetNumberFromUser();

// Distance traveled, in mm, since the last time ResetOdometer() was called.
// Positive numbers mean forwards, negative numbers mean iRobot has moved
// backwards.
//
// This information is updated based on information from the wheel sensors. Some
// motion functions use this to decide when to turn off the motors, and you can
// call this function after driving to see how far the iRobot actually drove.
int16_t OdometerDistance();

// Angle turned, in degrees, since the last time ResetOdometer() was called.
// Positive numbers mean counterclockwise (left), negative numbers mean
// clockwise (right).
//
// This information is updated based on information from the wheel sensors. Some
// motion functions use this to decide when to turn off the motors, and you can
// call this function after driving to see how far the iRobot actually drove.
int16_t OdometerAngle();

// The iRobot angle sensors are consistently low by about 5-15%. So what the
// iRobot body thinks is a 90 degree turn will be actually be closer to 100 or
// more. To compensate, the functions in iRobot.c will add or subtract (N% as
// needed, where N = 100/ODOMETER_ANGLE_CORRECTION. Setting the correction to 8
// will result in a 100/8 = 12.5% angle correction, setting it to 16 will result
// in 100/16 = 6.25% angle correction, and so on. You can comment out this next
// line of code to disable this correction behavior entirely.
// #define ODOMETER_ANGLE_CORRECTION (16)

// Reset the distance and angle odometers to zero. Most motion functions call
// this before turning on the wheel motors. 
void ResetOdometer();

// Check if the various buttons are being pressed right now.
// Returns true if the button is being pressed, false otherwise.
bool IsBlackButtonPressed();
bool IsPlayButtonPressed();
bool IsAdvanceButtonPressed();

// Check if the various bumpers are being pressed right now.
// Returns true if the bumper is being pressed, false otherwise.
bool IsLeftBumperPressed();
bool IsRightBumperPressed();
bool IsEitherBumperPressed(); // same as IsLeftBumperPressed() || IsRightBumperPressed()

// Get sensor readings from the infrared floor (aka "cliff") sensors. These
// might be useful for detecting light/dark patches on the floor, for following
// a dark line, etc. The range is 0 to 4095, with lower numbers meaning darker.
// See the instructor for help calibrating your robot. The sensor values are
// sensitive to lighting conditions, carpet, color, shadows, etc. If you rely on
// the floor sensors to detect changes in the floor color (e.g. white tape or
// paper on gray carpet, or dark tape on white tile floor), you will probably
// want to "auto-calibrate" the robot roughly as follows:
//    // The robot starts on white paper.
//    // First, we take readings to find the sensor value for light.
//    uint16_t light_value_left = FloorBrightnessLeft();
//    uint16_t light_value_right = FloorBrightnessRight();
//    // Next we move forward 30 cm so the robot is now on gray carpet.
//    Straight(30 * CENTIMETER);
//    // Then we look at the sensors again to find the value for dark.
//    uint16_t dark_value_left = FloorBrightnessLeft();
//    uint16_t dark_value_right = FloorBrightnessRight();
//    // Finally we take the average as our dividing point.
//    uint16_t medium_value_left = (light_value_left + dark_value_left) / 2;
//    uint16_t medium_value_right = (light_value_right + dark_value_right) / 2;
//    ...
//    // Later, we use the average to distinguish light and dark:
//    if (FloorBrightnessLeft() > medium_value_left) {
//		 // light floor color detected on left side
//	  } else {
//	     // dark floor color detected on left side
//	  }
//    if (FloorBrightnessRight() > medium_value_right) {
//		 // light floor color detected on right side
//	  } else {
//	     // dark floor color detected on right side
//	  }
uint16_t FloorBrightnessLeft();
uint16_t FloorBrightnessRight();
uint16_t FloorBrightnessFrontLeft();
uint16_t FloorBrightnessFrontRight();

// Get a sensor reading from the infrared wall sensor. The wall sensor is near
// the right front bumper. There is no sensor on the left side, so only
// right-side walls are detected. This function might be useful
// for detecting distance to a nearby wall, for following a wall, etc. The range
// is 0 to 4095, with lower numbers meaning nearer, but the result is not in
// millimeters or any other obvious unit.
uint16_t DistanceToRightWall();


/*
 * Infrared Communication.
 */


// Any robot can receive infrared signals, since the robot comes equiped with an
// omnidirectional infrared receiver (clear plastic dome mounted on the top of
// the robot, near the front bumper).
//
// The ability to send infrared signals only works if an IR LED and a 100ohm
// resistor are connected across pins 4 and 8 of the dock-side DB9 connector
// (the connector on the side of the green command module, not the top). Please
// ask your instructor before attempting to make these electrical connections.
//                         ___
//                        /   \  IR LED
//                        |___|
//                         | |       
//                short    | |    long
//                  leg    | \    leg
//                         |  |
//         /---------------|--|------\      .
//        /   (6)   (7)   (8) | (9)   \     .
//       /                 |  |        \    .  Dock-side DB 9 connector
//      /  (1)   (2)   (3) | (4)   (5)  \   .
//      \------------------|--|---------/
//                         |  \___________
//                         |              |
//                         |___/\/\/\/\___|   100 ohm resistor
//

// Broadcast one byte to all other nearby iRobots using the infrared channel.
// Sending data using this function requires the above electrical connections
// and IR LED.
void SendInfraredByte(uint8_t data);

// Check if iRobot detects a byte being broadcast by another iRobot or by a
// suitable IR remote. If a byte is being received, it is returned. If no data
// is being received, 255 is returned instead. Note that the robot will likely
// receive the same byte multiple times in a row, especially if the sender holds
// down the remote control button. This feature uses the iRobot's
// omnidirectional IR receiver.
uint8_t RecvInfraredByte();

// There is a remote control that can be used to manually send an infrared
// signal to the iRobots. Pressing a button on the remote control causes the
// corresponding byte to be repeatedly sent, for as long as you hold down the
// button.
#define REMOTE_LEFT    (129)
#define REMOTE_UPLEFT  (139) // unlabeled button between left and up
#define REMOTE_UP      (130)
#define REMOTE_UPRIGHT (140) // unlabeled button between right and up
#define REMOTE_RIGHT   (131)
#define REMOTE_RELEASE (141) // sent breifly when you release the left, up, or right buttons
#define REMOTE_PAUSE   (137)
#define REMOTE_SPOT    (132)
#define REMOTE_CLEAN   (136)
#define REMOTE_MAX     (133)
#define REMOTE_P       (138)


/*
 * Output.
 */

// There are five LEDs on the iRobot. We will call them LEDs 0 through 3 and P.
// P is the LED near the power button. It can be green, red, or a mix of colors.
// 3 is the LED near the black button on the command module.
// 2 is the LED near the red button on the command module.
// 1 is the LED near the middle "play" button on the iRobot.
// 0 is the LED near the rightmost "advance" button on the iRobot.

// Turn on or off the two small LEDs on the command module.
void SetModuleLED3(bool led3);
void SetModuleLED2(bool led2);
void SetModuleLEDs(bool led3, bool led2);

// Control the three LEDs on the iRobot itself, LED P, 1, and 0.
// ledPColor controls the color of the power button LED. 0 means green, 255
// means red, anything between gives a blend. You can use the GREEN, RED, and
// ORANGE constants for this parameter. ledPIntensity controls the brightness of
// the power button LED, from 0 (off) to 255 (full bright). You can use the
// BRIGHT or DIM constants for this parameter. led1 and led0 control the other
// two LEDs.
void SetRobotLEDs(uint8_t ledPColor, uint8_t ledPIntensity, bool led1, bool led0);

// Display a number from 0 to 15 by showing a bit pattern on LEDs 0 through 3.
// For numbers over 15, the P LED is set to bright red and all other LEDs are
// off. Otherwise, the P LED is set to bright green, and the pattern for LEDs 3,
// 2, 1, and 0 are follows, where "-" means OFF, and "o" means ON.
// 0: - - - -     4: - o - -      8: o - - -     12: o o - -     
// 1: - - - o     5: - o - o      9: o - - o     13: o o - o     
// 2: - - o -     6: - o o -     10: o - o -     14: o o o -     
// 3: - - o o     7: - o o o     11: o - o o     15: o o o o     
void DisplayNumber(uint8_t n);

// Display a number from 0 to 65535 by showing a series of bit patterns on LEDs
// 0 through 3. The bits are shown in four groups. Each of the four patterns is
// held for 1.5 seconds with a bright orange LED followed by 0.5 seconds with a
// dim orange LED. The four patterns represents numbers a, b, c, and d, in that
// order, such that n = (a*16*16*16) + (b*16*16) + (c*16) + d, using the same
// LED patterns as shown above for DisplayNumber().  After the last pattern, the
// small LEDs are turned off and the power LED is turned bright green.
void DisplayLargeNumber(uint16_t n);

// Define a song (a sequence of beeps/tones).
// There are slots for up to 16 different songs, numbered 0 to 15, and each slot
// holds up to 16 notes. Each note is a "MIDI note number":
//   31 (lowest pitch, a "low G" or "G1")
//   ...
//   59 (medium pitch, "B")
//   60 (medium pitch, "middle C" or "C4")
//   61 (medium pitch, "C#")
//   62 (medium pitch, "D")
//   ...
//   127 (highest pitch, a "high G" or "G9")
// Use 0 for silence. Google "MIDI note numbers" for a complete list. Each
// duration is specified in 1/64 fractions of a second, so 64 means one second,
// 32 means one half second, 16 means one quarter second, etc. Max duration is
// 255.
//
// In the simplest case, each song has at most 16 notes and fits in one slot. So
//   DefineSong(5, 13, short_song_notes, short_song_durations);
// would put 13 notes in slot 5, and you can later play this song with
//   PlaySong(5);
// However, an undocumented features seems to allow you to make a song with up to
// 32 notes by filling up two slots. So
//   DefineSong(6, 27, long_song_notes, long_song_durations);
// would put 16 notes in slot 6 and the remaining 11 notes in slot 7, and you
// can later play the song with
//   PlaySong(6);
// Since slot 7 gets filled in this example, you wouldn't be able to use slot 7
// for a song of its own.
void DefineSong(uint8_t song_number, uint8_t number_of_notes, uint8_t notes[], uint8_t durations[]);

// Play a song. The song number must have previously been created using DefineSong().
// This function returns 0 if the song started successfully. If another song is
// currently playing, then the new song is *not* started and this function
// returns -1 to indicate an error.
int8_t PlaySong(uint8_t song_number);

// Erase a song. If the song is currently playing, it will immediately stop.
// Actually, stopping a song that is currently playing is the only reason
// to use this function. You should call DefineSong() again before trying to
// play the same song again. There is a slight delay until the song actually
// stops playing, so if you call KillSong() then immediately call DefineSong(),
// you may end up playing the last half of the new song. Inserting a 
// delay would probably help avoid that situation.
void KillSong(uint8_t song_number);

// Find out if a song is currently playing. There is a slight delay between when
// you tell iRobot to play a song and when the song actually starts playing. So
// if you call PlaySong then immediately call IsPlayingSong it will probably
// return an incorrect answer.
bool IsPlayingSong();

// Find out which song is playing. If a song is currently playing, then
// that song's number is returned. Otherwise, if no song is currently playing,
// then this function returns -1;
int8_t CurrentPlayingSong();


/*
 * Motion.
 *
 * WARNING: Be careful not to damage the iRobot when using motion functions. The
 * iRobot may not stop the motors even if the wheels are jammed or it is about
 * to fall off a cliff. Depending on how the iRobot is set up, picking it up or
 * letting a wheel fall off a cliff may cause the robot to freeze.
 *
 * Many of the functions described here take distance, angle, radius, and other
 * similar prameters. See the comments above about "Useful constants" for more
 * information about distance, time, velocity, radius, and angle. measurements.
 *
 * Notes on distance and angle sensor accuracy: The distance and angles used for
 * most of the functions below are only approximate, for several reasons:
 *
 * (1) The sensors aren't perfect. First, the floor material (carpet, tile,
 *     wood, etc.) can affect things, since the robot is really just measuring
 *     how much the wheels have turned. 
 *
 *     To deal with this, you should always test, debug, and demo your robot on
 *     the same floor material. Switching to a new floor material on the day of
 *     your final demo is asking for trouble.
 *
 * (2) The robot tends to over-shoot a bit, since there is a slight lag time
 *     between when the robot senses it has traveled far enough and when the
 *     robot actually turns off the motors. When the robot is moving slowly,
 *     this might only be one or two millimeters (or degrees). If the robot is
 *     moving at high velocity, it might overshoot by a lot more.
 *
 *     To deal with this, you can drive slower when precise measurements are
 *     important. Or, just use shorter distances and angles in your code to
 *     compensate for the over-shoot. For example, with trial-and-error, you
 *     might find that a fast 90 degree turn on your robot can be accomplished
 *     by using a fast 75 degree turn in your code instead.
 *
 * (3) There is a bug in the iRobot firmware. Unfortunately, this bug can't be
 *     fixed. It's not even our fault! If you are interested, a short
 *     explanation of the bug is below, and iRobot.c contains extensive
 *     explanations. But the result is that you have to chose among three
 *     options for motions: you can have an accurate turn angle, but you can't
 *     use any other sensors while moving; you can have an accurate distance
 *     measurement, but you can't use any other sensors while moving; or you can
 *     use any sensor you want, but the distance and angle sensors will probably
 *     be inaccurate (uaully too low, resulting in overshoot).
 *
 *     To deal with this, a variety of motion functions are provided below. Some
 *     functions drive a specific distance, but they ignore the bumpers, angle,
 *     etc. These are the most accurate:
 *
 *       DriveDistance(v, r, d) - drive a specific distance straight or in an arc.
 *
 *     Other functions drive a specific turn angle, but ignore the bumpers,
 *     distance, etc.
 *
 *       Turn(a) - turns a specific angle in place.
 *       DriveAngle(v, r, a) - drives a specific angle in an arc.
 *
 *     And other functions use a mix of sensors, but won't be very accurate when
 *     it comes to distance or angle.
 *
 *       Backup() - go back a little.
 *       Forward(d) - go forward an approx. distance or until bumped.
 *       Straight(d) - go forward or back an approx. distance or until bumped.
 *       DriveOrBump(d) - drive an approx. distance or until bumped.
 *       TurnOrBump(a) - turn in place an approx. angle or until bumped.
 *       ArcOrBump(a) - drive in arc an approx. angle or until bumped.
 *
 * Details on the odometer bug:
 *
 * So, what's the bug anyway? It all comes down to rounding. The iRobot keeps
 * track of wheel rotations fairly accurately, in units of about 1/768ths of a
 * turn. When the green command module queries the latest odometer reading from
 * the iRobot, the iRobot convert from its internal units to degrees or
 * millimeters, as appropriate. The conversion involves division. And that
 * division uses integer division. And as we all know by now, integer division
 * *discards the fraction*, rounding the result down towards zero. So if the
 * robot has actually traveled 5.3 degrees, the decimal is discarded, and the
 * integer 5 is sent to the command module. That lost fraction means the command
 * module underestimates the actual distances and angles. The error accumulates
 * too: every time the sensor is queried, another fraction of a degree (or
 * millimeter) is lost. If we query the odometer rapidly, the error gets
 * magnified and we will vastly underestimate the actual distances and angles.
 * If we query the odometer only rarely, we will probably overshoot the desired
 * distance or angle. There is no happy medium. 
 *
 * So what can be done about the bug? Not much, sadly, unless the iRobot company
 * releases a firmware update (unlikely). There is a slight workaround. The
 * iRobot body has a built-in command to wait for a specific angle, and that
 * command is fairly accurate, but that command also ignores distances and other
 * sensors. Similarly, there is a built-in command to wait for a specific
 * distance, while ignoring angles and other sensors. So we can use one of those
 * techniques, or we can use the odometers and live with the errors.
 */

// Drive backwards at walking speed for about a quarter-second, then stop.
void Backup();

// Drive forward at walking speed for the specified distance, then stop. The
// distance is in millimeters and should be a postive number. A zero or negative
// distance will cause the robot to stop immediately and return true. If the
// robot bumps into something, this function stops immediately and returns
// false. Otherwise, if the robot travels the entire distance, the function
// returns true.
//
// Note: The distance here is only approximate, and usually the iRobot will
// travel a bit further than requested.
bool Forward(int16_t travel_distance); // approximate

// Go straight the specified distance (positive or negative) at walking speed,
// then stop. The distance is in millimeters. A value of 0 means go straight
// forever until we bump into something. If the robot bumps something, this
// function will stop early and return false. If the robot does not bump
// anything and has travelled the complete distance, then this funtion will
// return true. This function behaves just like Forward(), but it can go either
// forwards a set distance, forward forever, or backwards a set distance.
//
// Note: The distance here is only approximate, and usually the iRobot will
// travel a bit further than requested.
bool Straight(int16_t travel_distance); // approximate

// Drive the specified distance (positive or negative) at the specified velocity
// (positive or negative) and turning radius (positive or negative). A value of
// 0 for distance is not allowed. A value of 0 for velocity is not allowed. If
// the distance is positive, then the velocity should be positive as well.
// Similarly, if the distance is negative, then the velocity should be negative.
// There is no return value.
//
// This function behaves just like Straight(), but it does not stop when a bump
// is detected, nor is there a "forever" option (since there would be no way to
// stop the robot). Instead, this function always travels the specified
// distance. It also lets you specify a velocity and a turning radius.
//
// Velocity is given in mm/sec. Max velocity is 500 mm/sec. Use positive for
// forward, negative for backwards. Higher is faster. You can use the FAST,
// WALK, and SLOW constants for this parameter, either positive or negative.
//
// Turning radius is given in mm. Max turning radius is 2000 mm. Use positive
// for left turns, negative for right turns. Higher is straighter, lower is a
// tighter turn. 1 means turn in place counter-clockwise. -1 means turn in place
// clockwise. The special value 32768 means to drive straight. You can use the
// STRAIGHT_AHEAD, GENTLE_LEFT, SHARP_LEFT, SPIN_RIGHT, etc., constants for this
// parameter.
//
// The travel_distance parameter tells how far to drive before stopping, in mm.
// You can use any of the distance constants for this parameter, e.g. 3*FOOT. A
// value of zero is not allowed. The sign of the travel distance must agree with
// the sign of the velocity. So if you want to drive backwards fast for 3 feet,
// use velocity -FAST and travel_distance -3*FOOT.
//
// Note: This function usually achieves better accuracy for the distance than
// Forward(), Straight(), or any of the other motion functions. If you want to
// drive precise distances, try this function. Even so, the distances will not
// be perfect. 
void DriveDistance(int16_t velocity, int16_t radius, int16_t travel_distance); // accurate

// Turn in place to the left (positive, counter-clockwise) or right (negative,
// clockwise) at walking speed for the specified turn angle or until we bump
// something (whichever happens first), then stop. The turn angle is in degrees
// and should not be zero. If the robot bumps into something, this function
// stops immediately and returns false. Otherwise, if the robot completes the
// turn, the function returns true. 
//
// Note: The angle here is only approximate, and usually the iRobot will turn a
// bit further than requested.
bool TurnOrBump(int16_t turn_angle); // approximate

// Drive forward in an arc of the specified radius to the left or right at
// walking speed for the specified turn angle, then stop. The radius is in
// millimeters and should be a postive number for left arcs, negative for right
// arcs. The turn angle is in degrees and should not be zero. If the robot bumps
// into something, this function stops immediately and returns false. Otherwise,
// if the robot completes the arc, the function returns true.
//
// Note: The angle here is only approximate, and usually the iRobot will turn a
// bit further than requested.
bool ArcOrBump(int16_t radius, int16_t turn_angle);

// Turn in place to the left (positive, counter-clockwise) or right (negative,
// clockwise) at walking speed for the specified turn angle, then stop. The turn
// angle is in degrees and should not be zero. 
//
// This function behaves like a more accurate version of TurnOrBump(), but it
// does not stop when a bump is detected. 
//
// If turn_angle is positive, this is the same as
//   DriveAngle(WALK, SPIN_LEFT, turn_angle)
// If turn_angle is negative, this is the same as
//   DriveAngle(WALK, SPIN_RIGHT, turn_angle)
//
// Note: This function usually achieves better accuracy for the angle than the
// TurnLeft() function. If you want to turn precise angles in place, try this
// function. Even so, the angles will not be perfect. 
void Turn(int16_t turn_angle); // accurate

// Drive until the specified angle (positive or negative) at the specified
// velocity (positive or negative) and turning radius (positive or negative).
// The value of turn_angle should not be zero. A value of 0 for velocity is not
// allowed. There is no return value.
//
// This function behaves like Turn(), but it lets you specify a velocity and a
// turning radius, rather than using walking speed and turning in place.
//
// Velocity is given in mm/sec. Max velocity is 500 mm/sec. Use positive for
// forward, negative for backwards. Higher is faster. You can use the FAST,
// WALK, and SLOW constants for this parameter, either positive or negative.
//
// Turning radius is given in mm. Max turning radius is 2000 mm. Use positive
// for left turns, negative for right turns. Higher is straighter, lower is a
// tighter turn. 1 means turn in place counter-clockwise. -1 means turn in
// place clockwise. The special value 32768 means to drive straight. You can use
// the STRAIGHT_AHEAD, GENTLE_LEFT, SHARP_LEFT, SPIN_RIGHT, etc., constants for
// this parameter.
//
// The turn_angle parameter tells how far to turn before stopping, in degrees.
// The turn_angle should be positive for left turns, negative for right turns,
// but when driving backwards (negative velocity), the sign should be reversed.
// You can use an angle like 90*SPIN_RIGHT to turn 90 degrees right, or
// 45*SPIN_LEFT to turn 45 degrees left. 
//
// Note: This function usually achieves better accuracy for the angle than the
// TurnLeft() function. If you want to turn precise angles in place, try this
// function. Even so, the angles will not be perfect. 
void DriveAngle(int16_t velocity, int16_t radius, int16_t turn_angle); // accurate

// This is a "generic" drive function that go forwards, backwards, turn in place
// in either direction, or arc forwards or backwards in either direction. It
// drives at the specified velocity and turning radius, then stops once the
// iRobot has traveled the specified distance, or the iRobot has turned the
// specified angle, or the robot has bumped into something. If the robot bumps
// something, this function will stop immediately and return false. If the robot
// does not bump anything and has travelled the complete distance (or turn
// angle), then this funtion will return true.
//
// Velocity is given in mm/sec. Max velocity is 500 mm/sec. Use positive for
// forward, negative for backwards. Higher is faster. 0 means stop the motors.
// You can use the FAST, WALK, and SLOW constants for this parameter, either
// positive or negative.
//
// Turning radius is given in mm. Max turning radius is 2000 mm. Use positive
// for left turns, negative for right turns. Higher is straighter, lower is a
// tighter turn. 1 means turn in place counter-clockwise. -1 means turn in place
// clockwise. The special value 32768 means to drive straight. You can use the
// STRAIGHT_AHEAD, GENTLE_LEFT, SHARP_LEFT, SPIN_RIGHT, etc., constants for this
// parameter.
//
// The travel_distance parameter tells how far to drive before stopping, in mm.
// You can use any of the distance constants for this parameter, e.g. 3*FOOT. A
// value of 0 means to go forever until it bumps into something or we get to the
// correct angle. The sign of the travel distance must agree with the sign of
// the velocity. So if you want to drive backwards fast for 3 feet, use velocity
// -FAST and travel_distance -3*FOOT.
//
// The turn_angle parameter tells how far to turn before stopping, in degrees. A
// value of 0 means to go forever until it bumps into something or we get to the
// correct distance. The turn_angle should be positive for left turns, negative
// for right turns, but when driving backwards (negative velocity), the sign
// should be reversed. You can use an angle like 90*SPIN_RIGHT to turn 90
// degrees right, or 45*SPIN_LEFT to turn 45 degrees left. 
//
// Note: The distances and angles here are only approximate, and usually the
// iRobot will turn or drive a bit further than requested.
bool DriveOrBump(int16_t velocity, int16_t radius, int16_t travel_distance, int16_t turn_angle); // approximate

// Stop all motors immediately.
void Stop();

// Stop all motors after the robot has turned the specified number of degrees.
// This is used for accurate turns.
void StopAfterAngle(int16_t turn_angle);

// Stop all motors after the robot has moved the specified distance, in mm.
// This is used for accurate distances.
void StopAfterDistance(int16_t distance);

// Start moving at the specified velocity and turning radius. Positive velocity
// means forward, negative means backwards. Unlike the motion functions above,
// this function starts the iRobot moving and it keeps moving even after the
// function returns. The iRobot keeps going until you call Stop(),
// StopAfterAngle(), or StopAfterDistance() to make it stop. In contrast, the
// other motion functions above do not return until some event occurs to cause
// the iRobot to stop, such as a bumper press or having reached a specified
// distance.
//
// Here is an example of one way to use this function.
//        ResetOdometer();
//        StartMoving(WALK, STRAIGHT_AHEAD); // start moving
//        while (OdometerDistance() < 1 * METER) { // keep going for roughly 1 METER
//          // See if we bumped something, and if so, then break out of loop
//          if (IsEitherBumperPressed())
//				break;
//        }
//        Stop(); // stop the robot when the loop finishes
// This code is, in fact, almost identical to what the Forward() function above
// does. See Section 1 of iRobot.c for the actual code.
//
// Here is another example of how to use this function.
//        StartMoving(WALK, STRAIGHT_AHEAD); // start moving
//        Delay(1*SECOND); // wait one second
//        Stop();
// This code is, in fact, almost identical to what the Backup() function
// above does. See Section 1 of iRobot.c for the actual code.
//
// Here is yet another example of how to use this function.
//        ResetOdometer();
//        StartMoving(WALK, STRAIGHT_AHEAD); // start moving
//        StopAfterDistance(1*METER); // keep going for 1 METER, accurately, then stop
// This code is, in fact, almost identical to what the DriveDistance() function
// above does. See Section 1 of iRobot.c for the actual code.
void StartMoving(int16_t velocity, int16_t radius);


/*
 * Random useful stuff.
 */

// Return a random number in the range 0 to max_num (inclusive). Thus
// PickRandomUnsigned(15) will pick a number n such that 0 <= n <= 15. This
// function is somewhat slow because finding random numbers is computationally
// expensive. In the best case, the range of numbers should be a power of 2.
// That is, max_num should be 1, 3, 7, 15, 31, etc., i.e. if max_num+1 is a
// power of 2. Any other value of max_num will make this function even slower. 
uint16_t PickRandomAtMost(uint16_t max_num);

// Return a random number in the range min_num to max_num (inclusive on both
// extremes). This only works correctly for reasonably small values, e.g. in the
// range of plus or minus a few thousand. Like PickRandomAtMost(), this function
// is slow and it is best if the range of numbers (max_num-min_num+1) is a power
// of 2.
int16_t PickRandomNumber(int16_t min_num, int16_t max_num);

// Return a random angle, in degrees, in the range plus or minus 180 degrees.
int16_t PickRandomAngle();

// Return a random angle, in degrees, at least plus or minus 60 degrees, but no
// more than plus or minus 180 degrees. The random is somewhat biased towards
// the large angles. This could be useful for turning around and going randomly
// "away" from an obstacle after a bump.
int16_t PickRandomBackwardsAngle();

/*
 * Debugging stuff. Don't use these.
 */
#ifdef IROBOT_DEBUG
void _ArduinoSend(uint8_t data);
void _ArduinoSendString(const char *buf);
void _ArduinoSendInt(int16_t x);
void _ArduinoSendUInt(uint16_t x);
uint16_t _OdometerReadings();
#endif // IROBOT_DEBUG

#endif  // _IROBOT_H_
