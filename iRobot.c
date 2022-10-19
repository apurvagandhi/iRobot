/*******************************************************************************
 * iRobot.c, version 0.5
 *
 * Implementation of the simple high-level iRobot API define in iRobot.h.
 * Do NOT put your main function in this file. Put main in a separate file.
 *
 * This file is split into three sections:
 *
 * Section 1. Useful code you might want to look at. These functions are simple
 *   enough that any student in CSCI 131 should be able to read and understand
 *   them. Feel free to copy-and-paste pieces of this code into your own
 *   programs, or change the code right in this file if you like.
 *
 * Section 2. These functions are a bit tricky, but might still be useful.
 *   Unless you are very sure of yourself, you probably want to simply ignore
 *   this code.
 *
 * Section 3. Low-level internal code. This code is subtle and tricky, and if
 *   you change any of this code you are very likely to break something. There
 *   is not much reason for anyone to look at this code unless you simply want
 *   to get a glimpse of what the iRobot command module is doing behind the
 *   scenes to control the iRobot body.
 *
 * Modified by kwalsh@cs.holycross.edu, based closely on the original RobotAPI.c
 * Version History:
 *   Pre Fall 2012
 *      - Origional RobotAPI.c, presumably from the iRobot company.
 *   Pre Fall 2012
 *      - Modified by LKing and Christina D'Ambrosio'11 summer 2008-2010.
 *   Fall 2012 (kwalsh)
 *      - Renamed files, split into proper implementation and header files.
 *      - Renamed most variables and functions to be more intuitive.
 *      - Added unit constants.
 *      - Added lots of comments.
 *      - Eliminate interrupts for serial receive. It simply wasn't necessary.
 *   Spring 2013 (kwalsh)
 *      - Turn on SafeMode rather than FullMode, and eliminated much of the
 *        now-obsolete code that tried to deal with dangerous situations.
 *      - Simplify and calibrate the Drive() function.
 *      - Added a callback to Drive() function.
 *      - Expose a few new functions, remove a few obsolete functions.
 *      - Proper handling of atomicity for multi-byte variables accessed in ISR.
 *      - Added heartbeat() and new uptime functions.
 *   Fall 2014 (kwalsh)
 *      - Made constants UPPER_CASE intead of TitleCase.
 *      - Made functions TitleCase instead of camelCase.
 *      - Added HaveBumped() and HaveArrived() functions to replace the old Drive()
 *        return values.
 *      - Reorganized code into three sections.
 *      - Try SensorStream command with serial RX interrupts.
 *      - Try script commands for better angle measurements.
 *      - Use ATOMIC_BLOCK() primitives.
 *      - Add Arduino logging code (software serial hack).
 *   Fall 2017 (kwalsh)
 *      - Changed back to ISR again, during radius-to-nuc transition.
 *******************************************************************************/

#include "iRobot.h"

/*******************************************************************************
 * Section 1. Simple and useful code you might want to look at. Feel free to
 *   copy-and-paste pieces of this code into your own programs, or change the
 *   code right here in this file if you like.
 *******************************************************************************/

void Backup()
{
	StartMoving(-WALK, STRAIGHT_AHEAD);
	Delay(250); // Wait about a quarter of a second (250 milliseconds).
	Stop();
}

bool Forward(int16_t travel_distance)
{
	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(WALK, STRAIGHT_AHEAD);

	// keep driving until we bump or we have gone far enough
    while (true) {
        // Check if we have gone far enough forward
        if (travel_distance <= OdometerDistance())
            break;
        // Check if we have bumped something
		if (IsEitherBumperPressed()) {
			Backup();
			return false;
		}
	}

	Stop();
    return true;
}

bool Straight(int16_t travel_distance)
{
    int16_t velocity;
    if (travel_distance >= 0) {
        velocity = WALK;
    } else {
        velocity = -WALK;
    }

	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(velocity, STRAIGHT_AHEAD);

	// keep driving until we bump or we have gone far enough
    while (true) {
        // Check if we have gone far enough forward
        if (0 < travel_distance && travel_distance <= OdometerDistance())
            break;
        // Check if we have gone far enough backward
        if (0 > travel_distance && travel_distance >= OdometerDistance())
            break;
        // Check if we have bumped something
		if (IsEitherBumperPressed()) {
			Backup();
			return false;
		}
	}

	Stop();
    return true;
}

bool TurnOrBump(int16_t turn_angle)
{
    // This function is identical to ArcOrBump(), but using SPIN_LEFT or
    // SPIN_RIGHT as the radius, as appropriate.
    if (turn_angle < 0)
        return ArcOrBump(SPIN_RIGHT, turn_angle);
    else
        return ArcOrBump(SPIN_LEFT, turn_angle);
}

bool ArcOrBump(int16_t radius, int16_t turn_angle)
{
	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(WALK, radius);

	// keep driving until we bump or we have gone far enough
    while (true) {
        // Check if we have turned far enough left 
	    if (0 < turn_angle && turn_angle <= OdometerAngle())
            break;
        // Check if we have turned far enough right 
		if (0 > turn_angle && turn_angle >= OdometerAngle())
            break;
        // Check if we have bumped something
		if (IsEitherBumperPressed()) {
			Backup();
			return false;
		}
	}

	Stop();
    return true;
}

void Turn(int16_t turn_angle)
{
    // This function is identical to DriveAngle(), but using WALK as the
    // velocity, and using SPIN_LEFT or SPIN_RIGHT as the radius, as
    // appropriate.
	if (turn_angle < 0)
		DriveAngle(WALK, SPIN_RIGHT, turn_angle);
	else
		DriveAngle(WALK, SPIN_LEFT, turn_angle);
}

void DriveAngle(int16_t velocity, int16_t radius, int16_t turn_angle)
{
	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(velocity, radius);
    // stop when we have turned the desired angle
    StopAfterAngle(turn_angle);
}

void DriveDistance(int16_t velocity, int16_t radius, int16_t travel_distance)
{
	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(velocity, radius);
    // stop when we have moved the desired distance
    StopAfterDistance(travel_distance);
}

void Delay(uint16_t time_ms)
{
	StartTimer(time_ms);
	while (! IsTimerExpired()) {
		/* do nothing */
    }
}

bool WaitForBump(uint16_t time_ms)
{
	StartTimer(time_ms);
	while (! IsTimerExpired()) {
		if (IsEitherBumperPressed()) {
			Backup();
			return true;
		}
	}
	return false;
}


/*******************************************************************************
 * Section 2. These functions are a bit tricky, but might still be useful.
 *   Unless you are very sure of yourself, you probably want to simply ignore
 *   this code.
 *******************************************************************************/

bool DriveOrBump(int16_t velocity, int16_t radius, int16_t travel_distance, int16_t turn_angle)
{
	// reset the distance and angle counters then start driving
	ResetOdometer();
	StartMoving(velocity, radius);

	// keep driving until we bump or we have gone far enough
    while (true) {
        // Check if we have gone far enough forward
        if (0 < travel_distance && travel_distance <= OdometerDistance())
            break;
        // Check if we have gone far enough backward
        if (0 > travel_distance && travel_distance >= OdometerDistance())
            break;
        // Check if we have turned far enough left 
	    if (0 < turn_angle && turn_angle <= OdometerAngle())
            break;
        // Check if we have turned far enough right 
		if (0 > turn_angle && turn_angle >= OdometerAngle())
            break;
        // Check if we have bumped something
		if (IsEitherBumperPressed()) {
			Backup();
			return false;
		}
	}

	Stop();
    return true;
}

int16_t PickRandomNumber(int16_t min_num, int16_t max_num) {
	return min_num + PickRandomAtMost((uint16_t)(max_num - min_num));
}

int16_t PickRandomBackwardsAngle()
{
	int16_t r_angle = PickRandomAngle();
	if (-60 < r_angle && r_angle < 0)
		r_angle += 180;
	else if (0 <= r_angle && r_angle < 60)
		r_angle -= 180;
	return r_angle;
}

// Resolution for polling the black button is 100 ms, or one tenth of a second.
static const uint16_t BUTTON_POLL_TIME = 100;

uint16_t WaitForBlackButton() {
	uint16_t time_ms = 0;
	// Stop();
	while (!IsBlackButtonPressed()) {
		Delay(BUTTON_POLL_TIME);
    }
	while (IsBlackButtonPressed()) {
		Delay(BUTTON_POLL_TIME);
		time_ms += BUTTON_POLL_TIME;
	}
	return time_ms;
}

bool GetBitFromUser() {
	uint16_t time_ms = 0;
	// Stop();
	while (!IsBlackButtonPressed()) {
		Delay(BUTTON_POLL_TIME);
		time_ms += BUTTON_POLL_TIME;
		if (time_ms > 2 * SECOND)
			return false; // time is up and no button press yet
	}
	// button has been pressed, wait until they stop pressing then return
	while (IsBlackButtonPressed())
		Delay(BUTTON_POLL_TIME);
	return true;
}

uint8_t GetNumberFromUser() {
	uint16_t time_ms = 0;
	uint8_t counter = 0;

	// Stop();
	DisplayNumber(counter);

	while (!IsBlackButtonPressed())
		Delay(BUTTON_POLL_TIME);

	while (IsBlackButtonPressed()) {
		Delay(BUTTON_POLL_TIME);
		time_ms += BUTTON_POLL_TIME;
		if (time_ms > SECOND) {
			counter = (counter + 1) & 0xF;
			DisplayNumber(counter);
			time_ms = 0;
		}
	}
	return counter;
}

void DisplayNumber(uint8_t n) {
	if (n > 15) {
		SetModuleLEDs(false, false);
		SetRobotLEDs(RED, BRIGHT, false, false);
	} else {
		SetModuleLEDs(!!(n&8), !!(n&4));
		SetRobotLEDs(GREEN, BRIGHT, !!(n&2), !!(n&1));
	}
}

void DisplayLargeNumber(uint16_t n) {
    uint8_t parts[] = {
        (uint8_t)((n>>12) & 0xf),
        (uint8_t)((n>>8) & 0xf),
        (uint8_t)((n>>4) & 0xf),
        (uint8_t)(n & 0xf)
    };
    for (uint8_t i = 0; i < 4; i++) {
        n = parts[i];
		SetModuleLEDs(!!(n&8), !!(n&4));
		SetRobotLEDs(ORANGE, BRIGHT, !!(n&2), !!(n&1));
        Delay(1500);
		SetRobotLEDs(ORANGE, DIM, !!(n&2), !!(n&1));
        Delay(500);
    }
    SetRobotLEDs(GREEN, BRIGHT, false, false);
    SetModuleLEDs(false, false);
}

void Stop() {
	StartMoving(0, STRAIGHT_AHEAD);
}

/*******************************************************************************
 * Section 3. Low-level internal code. This code is subtle and tricky, and if
 *   you change any of this code you are very likely to break something. There
 *   is not much reason for anyone to look at this code unless you simply want
 *   to get a glimpse of what the iRobot command module is doing behind the
 *   scenes to control the iRobot body.
 *******************************************************************************/

#include <util/atomic.h>

// Join bytes x and y into a 16-bit value.
#define BYTE_PAIR(x, y) (((x) << 8) | (y))

// Tick ISR delay, in ms.
#define TICK_INTERVAL (1) // 1000 hz
// #define TICK_INTERVAL (3) // 300 hz, appropriate for Arduino baud
// #define TICK_INTERVAL (10) // 100 hz

// Enable serial output to an attached Arduino. 
// Comment out this line to disable it, uncomment to enable it.
// #define ARDUINO_SERIAL_ENABLED

// How many ticks are in t msec, rounded up to nearest tick.
#define MSEC_TO_TICKS(t) (((t)+(TICK_INTERVAL-1))/TICK_INTERVAL)

// Private global variables. Volatile ones are used within timer ISR.
static volatile uint16_t _timer_cnt = 0; // ticks left before expiration
static volatile uint8_t _timer_expired = 0; // set to 1 whenever _timer_cnt == 0
static volatile uint16_t _uptime_s = 0; // seconds since startup
static volatile uint16_t _uptime_ms = 0; // additional msec since startup
static volatile int16_t _distance = 0; // mm driven since last reset
static volatile int16_t _angle = 0; // degrees turned since last reset
static uint8_t _last_song; // used for KillSong() and CurrentPlayingSong()

// Private global variables for debugging.
static uint16_t _telemetry_readings = 0; // number of telemetry readings since last reset
// uint8_t _telemetry_angle_bucket[21]; // -10..10

// Prototypes for private low-level functions.
void _byteTx(uint8_t value); // send value from command module to iRobot 
void _initializeCommandModule();
void _powerOnRobot();
void _setSerialSpeed(uint8_t baud);
uint8_t _byteRx(void);
void _byteRxFlush(void);

// Sanity check: maximum expected change in distance between sensor readings.
static const int16_t MAX_DISTANCE_CHANGE = 20000;

// Sanity check: maximum expected change in angle between sensor readings.
static const int16_t MAX_ANGLE_CHANGE = 20000;

void WakeRobot() {
	_initializeCommandModule();
	LEDBothOff();
	_powerOnRobot();
	_byteTx(CmdStart);
    // Default baud upon startup is 57600. We use something lower than this to
    // try and reduce errors. But we do still use a higher baud than the 28800
    // baud recommended in the iRobot Create manual to get a larger byte budget
    // for sensing.
	// _setSerialSpeed(Baud57600); 
	_setSerialSpeed(Baud38400); 
	// _setSerialSpeed(Baud28800); // Recommended, but only about twice our budget.
	// _setSerialSpeed(Baud14400); // Too slow for our sensor budget.
	// _setSerialSpeed(Baud9600); //  Too slow for our sensor budget.
	_byteTx(CmdControl);
	_byteTx(CmdSafe); // CmdSafe or CmdFull
	// Done with set up.

	Stop();

	SetRobotLEDs(GREEN, BRIGHT, false, false);
	SetModuleLEDs(false, false);
}

uint16_t UptimeSeconds()
{
	uint16_t s;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
        s = _uptime_s;
    }
	return s;
}

uint32_t UptimeMilliseconds()
{
	uint16_t ms, s;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
        ms = _uptime_ms;
        s = _uptime_s;
    }
	return (1000*(uint32_t)s) + ms;
}

uint8_t Heartbeat()
{
	return (uint8_t)(_uptime_ms >> 8); // high byte only, so safe to do outside of ISR
}

// The iRobot sensor system has a 15ms window in which it responds to some
// (all?) commands. The manuals are slightly ambiguous about what the real
// constraints are:
//
//    "Create updates its sensors and replies to sensor requests at a rate
//    of 67 Hz, or every 15 milliseconds. Do not send sensor requests faster
//    than this."
//
//    [Discussing sense commands] "The following commands let you read the
//    state of Create’s built-in sensors, digital and analog inputs, and
//    some internal state variables. Create updates these values internally
//    every 15 ms. Do not send these commands more frequently than that."
//
//    [Discusssing streaming command] "It is up to you not to request more
//    data than can be sent at the current baud rate in the 15 ms time slot.
//    For example, at 57600 baud, a maximum of 86 bytes can be sent in 15
//    ms: 15 ms / 10 bits (8 data + start + stop) * 57600 = 86.4 If more
//    data is requested, the data stream will eventually become corrupted.
//    This can be confirmed by checking the checksum.
//
// Speculation: The iRobot queues incomming commands, and has an ISR running
// at 15ms intervals to consume and responds to any queued commands. If a
// the response to a sense command takes, say, 5ms to send, then the iRobot
// can respond to at most 3 sense commands in each 15ms window. If you send
// sense requests at a rate higher than that, the iRobot's input queue will
// eventually overflow and cause corruption.
//
// Speculation: It isn't clear if the hypothetical iRobot ISR must
// completely finish all of its processing before the end of the 15ms
// interval. Presumably yes, which would mean you must not send a "large"
// sense command whose reply would take more than 15 ms to send. This amount
// of data that represents depends on the baud rate:
//
//                             (52 bytes)  (26 bytes)  (10 bytes)  (6 bytes)
//   Baud   Reply Window Size   All         Priority    Safety      Telemetry 
//   57600  86 bytes maximum    9.0 ms      4.5 ms      1.7 ms      1.0 ms
//   28800  43 bytes maximum   18.1 ms      9.0 ms      3.5 ms      2.1 ms
//   14400  21 bytes maximum   36.1 ms     18.1 ms      6.9 ms      4.2 ms
//
// Observation: The Stream command can be used request all interesting sensor
// data be sent automatically in every 15ms interval. We could request:
//   bytes  sensor
//   6      GroupTelemetry (IRChar, Dist, Ang, Button)
//   1      BumpDrop
//   14     GroupEnviro (raw wall and floor/cliff sensors)
//   1      OISong
// The response will contain a header (2 bytes), sensor packet IDs (4 bytes),
// sensor data (22 bytes), and a checksum (1 byte). This is 29 bytes. At 28800
// baud, it uses somewhat more than half the 15 ms budget. Unfortunately,
// requesting distance and angle data at 15 ms intervals leads to massive
// rounding errors (this is a known issue).
//
// Strategy: We cache the results of the most recent sensor readings for 15ms.
// New sensor readings always check this cache first. In addition, distance and
// angle odometer readings are separately cached for some time, in an attempt to
// avoid some rounding errors.

#define SENSOR_RX_DataBumpDrop      0 // BumpDrop
#define SENSOR_RX_DataIRChar        1 // IRChar
#define SENSOR_RX_DataButton        2 // Button
#define SENSOR_RX_DataWallSig       3 // Enviro...
#define SENSOR_RX_DataWallSig0      4
#define SENSOR_RX_DataCliffLSig     5
#define SENSOR_RX_DataCliffLSig0    6
#define SENSOR_RX_DataCliffFLSig    7
#define SENSOR_RX_DataCliffFLSig0   8
#define SENSOR_RX_DataCliffFRSig    9
#define SENSOR_RX_DataCliffFRSig0   10
#define SENSOR_RX_DataCliffRSig     11
#define SENSOR_RX_DataCliffRSig0    12
#define SENSOR_RX_DataInputs        13
#define SENSOR_RX_DataAInput        14
#define SENSOR_RX_DataAInput0       15
#define SENSOR_RX_DataChAvailable   16
#define SENSOR_RX_DataOISong        17 // OISong
#define SENSOR_RX_SIZE              18 // buffer size

#define SENSOR_RX_EXPIRATION 100 // msec (will be converted to ticks)

static uint8_t _sensor_rx[SENSOR_RX_SIZE];
static volatile uint8_t _sensor_rx_timer = 0; // ticks

void _refreshSensors()
{
    // Check cache
    if (_sensor_rx_timer > 0)
        return;
    
    // Flush garbage from queue
    _byteRxFlush();

    // Query sensors
    _byteTx(CmdSensorList);
    _byteTx(5); 
    _byteTx(SensorIDBumpDrop); 
    _byteTx(SensorIDIRChar); 
    _byteTx(SensorIDButton); 
    _byteTx(SensorIDGroupEnviro); 
    _byteTx(SensorIDOISong); 

	for (uint8_t i = 0; i < SENSOR_RX_SIZE; i++)
		_sensor_rx[i] = _byteRx();
	
    _sensor_rx_timer = MSEC_TO_TICKS(SENSOR_RX_EXPIRATION);
}

#define TELEMETRY_RX_DataDist   0
#define TELEMETRY_RX_DataDist0  1
#define TELEMETRY_RX_DataAng    2
#define TELEMETRY_RX_DataAng0   3
#define TELEMETRY_RX_SIZE       4 // buffer size

// todo: make telemetry expiration dependent on velocity?
#define TELEMETRY_RX_EXPIRATION 0 // 15 // msec (will be converted to ticks)

static uint8_t _telemetry_rx[TELEMETRY_RX_SIZE];
static volatile uint8_t _telemetry_rx_timer = 0; // ticks

void _refreshTelemetry()
{
    // Check cache
    if (_telemetry_rx_timer > 0)
        return;

    // Flush garbage from queue
    _byteRxFlush();

    // Query sensors
    _byteTx(CmdSensorList);
    _byteTx(2); 
    _byteTx(SensorIDDist); 
    _byteTx(SensorIDAng); 

	for (uint8_t i = 0; i < TELEMETRY_RX_SIZE; i++)
		_telemetry_rx[i] = _byteRx();

    int16_t change;

    // The iRobot firmware apparently drops any fractional units on every
    // telemetry sensor request, leading to a consistent under-estimation, and
    // this is especially bad for angles. To compensate, we might try to add 0.5
    // units (degrees or mm) to every sensor reading, or 0.25 units. Actaully,
    // we would add 1 unit roughly every second (or fourth) sensor reading. this
    // doesn't work, because the readings are consistently biased. For example,
    // in a slow turn, the actual angle might be about 1.3 degrees each reading.
    // If we do 72 consecutive readings and get 1.0 degrees each time,
    // compensated up to 1.25, that gives 90 estimated degrees and 93.6 actual
    // degrees. With a slightly faster turn, it might be 1.8 actual degrees each
    // reading. So 72 consecutive readings will still give the same estimate of
    // 90 degrees, even though the actual angle will be 129.6 degrees.
    //
    // Instead, we might compensate by 0.5 each time, but aim for roughtly 5
    // degrees per sensor measurement. If the sensor measurement is 5,
    // compensated up to 5.5, then the actual error in sensing is no worse than
    // 0.5, or relative error of about 10%.  But there is also up to 5 degrees
    // of overshoot (or perhaps 2.5 degrees undershoot/overshoot) when we decide
    // to stop. On a 90 degree turn, that could be as bad as 15 degrees. Which
    // is no better than before.
    // bool compensate = false; // ((_telemetry_readings % 4) == 0);

    change = (int16_t)BYTE_PAIR(_telemetry_rx[TELEMETRY_RX_DataDist], _telemetry_rx[TELEMETRY_RX_DataDist0]);
    /* if (compensate && change > 0) {
        change += 1;
    } else if (compensate && change < 0) {
        change -= 1;
    } */
    if (-MAX_DISTANCE_CHANGE <= change && change <= MAX_DISTANCE_CHANGE)
        _distance += change;

    change = (int16_t)BYTE_PAIR(_telemetry_rx[TELEMETRY_RX_DataAng], _telemetry_rx[TELEMETRY_RX_DataAng0]);
    /* uint8_t bucket;
    if (-10 <= change && change <= 10) {
        bucket = 10 + change;
    } else if (change < 0) {
        bucket = 0;
    } else {
        bucket = 20;
    }
    _telemetry_angle_bucket[bucket]++;

    if (compensate && change > 0) {
        change += 1;
    } else if (compensate && change < 0) {
        change -= 1;
    } */
    if (-MAX_ANGLE_CHANGE <= change && change <= MAX_ANGLE_CHANGE)
        _angle += change;

    _telemetry_readings++;
    _telemetry_rx_timer = MSEC_TO_TICKS(TELEMETRY_RX_EXPIRATION);
}

// This function waits until the wheels have stopped moving.
void WaitUntilStopped()
{
    // Sadly, this next line does not do what we want.
    // _refreshTelemetry(); // This should block until "Wait Distance" completes.
   
    // Instead, wait until both wheels are powered off.
    uint8_t wheel_vel = 0;
    do {
        _byteTx(CmdSensorList);
        _byteTx(2); 
        _byteTx(SensorIDVelR); 
        _byteTx(SensorIDVelL); 
        for (uint8_t i = 0; i < 4; i++)
            wheel_vel |= _byteRx();
    } while (wheel_vel != 0);
}

// This function uses the "wait angle" command, followed immediately by a stop
// command which the iRobot will perform only after the "wait angle" is
// finished. While this is happening, we repeatedly look at the wheel velocities
// to see if the robot has stopped moving.
// Note: the manual implies that we could instead use a single command, e.g.
// reading the telemetry data, and that the iRobot will only respond once the
// "wait angle" command is completed.
//   "Until Create turns through the specified angle, its state does not
//    change, nor does it react to any inputs, serial or otherwise."
// This does not appear to be the case: the sensor reading is returned
// immediately, even though the "wait angle" hasn't been processed.
void StopAfterAngle(int16_t turn_angle)
{
#ifdef ODOMETER_ANGLE_CORRECTION
    // Correct for bad angles.
    turn_angle -= turn_angle/ODOMETER_ANGLE_CORRECTION;
#endif

	_byteTx(CmdWaitAngle);
	_byteTx((uint8_t)((turn_angle >> 8) & 0x00FF));
	_byteTx((uint8_t)(turn_angle & 0x00FF));
	Stop();
    WaitUntilStopped();
}

// This function uses the "wait distance" command, followed immediately by a stop
// command which the iRobot will perform only after the "wait distance" is
// finished. While this is happening, we repeatedly look at the wheel velocities
// to see if the robot has stopped moving.
void StopAfterDistance(int16_t distance)
{
	_byteTx(CmdWaitDist);
	_byteTx((uint8_t)((distance >> 8) & 0x00FF));
	_byteTx((uint8_t)(distance & 0x00FF));
	Stop();
    WaitUntilStopped();
}

void ResetOdometer() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
        _telemetry_rx_timer = 0;
    }
    _refreshTelemetry();
    _distance = 0;
    _angle = 0;
    _telemetry_readings = 0;
	/* ATOMIC_BLOCK(ATOMIC_FORCEON) {
        _telemetry_rx_timer = 0;
    } */
}

int16_t _OdometerReadings() {
    return _telemetry_readings;
}

int16_t OdometerDistance() {
    _refreshTelemetry();
    return _distance;
}

// Wait for then receive a byte over the serial port
uint8_t _byteRx(void)
{
	while (!(UCSR0A & _BV(RXC0)))
		;
	return UDR0;
}

void _byteRxFlush(void)
{
	uint8_t junk;
	while ((UCSR0A & _BV(RXC0))) {
		junk = UDR0;
        (void)junk;
    }
}

int16_t OdometerAngle() {
    _refreshTelemetry();
#ifdef ODOMETER_ANGLE_CORRECTION
    return _angle + _angle/ODOMETER_ANGLE_CORRECTION;
#else
    return _angle;
#endif
}

bool IsPlayButtonPressed() { _refreshSensors(); return !! (_sensor_rx[SENSOR_RX_DataButton] & ButtonPlay); }
bool IsAdvanceButtonPressed() { _refreshSensors(); return !! (_sensor_rx[SENSOR_RX_DataButton] & ButtonAdvance); }

bool IsLeftBumperPressed() { _refreshSensors(); return !! (_sensor_rx[SENSOR_RX_DataBumpDrop] & BumpLeft); }
bool IsRightBumperPressed() { _refreshSensors(); return !! (_sensor_rx[SENSOR_RX_DataBumpDrop] & BumpRight); }
bool IsEitherBumperPressed() { _refreshSensors(); return !! (_sensor_rx[SENSOR_RX_DataBumpDrop] & BumpEither); }

uint16_t FloorBrightnessLeft() { 
    _refreshSensors();
    return (uint16_t)BYTE_PAIR(_sensor_rx[SENSOR_RX_DataCliffLSig], _sensor_rx[SENSOR_RX_DataCliffLSig0]);
}
uint16_t FloorBrightnessRight() {
    _refreshSensors();
    return (uint16_t)BYTE_PAIR(_sensor_rx[SENSOR_RX_DataCliffRSig], _sensor_rx[SENSOR_RX_DataCliffRSig0]);
}
uint16_t FloorBrightnessFrontLeft() {
    _refreshSensors();
    return (uint16_t)BYTE_PAIR(_sensor_rx[SENSOR_RX_DataCliffFLSig], _sensor_rx[SENSOR_RX_DataCliffFLSig0]);
}
uint16_t FloorBrightnessFrontRight() {
    _refreshSensors();
    return (uint16_t)BYTE_PAIR(_sensor_rx[SENSOR_RX_DataCliffFRSig], _sensor_rx[SENSOR_RX_DataCliffFRSig0]);
}

uint16_t DistanceToRightWall() {
    _refreshSensors();
    return (uint16_t)BYTE_PAIR(_sensor_rx[SENSOR_RX_DataWallSig], _sensor_rx[SENSOR_RX_DataWallSig0]);
}

void StartTimer(uint16_t time_ms)
{
    uint16_t ticks = MSEC_TO_TICKS(time_ms);
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
        _timer_cnt = ticks;
        _timer_expired = 0;
    }
}

bool IsTimerExpired() {
    return (_timer_expired != 0);
}

bool IsBlackButtonPressed() { return UserButtonPressed(); }

void SendInfraredByte(uint8_t data)
{
	_byteTx(CmdIRChar);
	_byteTx(data); 
}

uint8_t RecvInfraredByte() { _refreshSensors(); return (uint8_t)(_sensor_rx[SENSOR_RX_DataIRChar]); }

void SetModuleLED3(bool led3) {
	if (led3) LED2On();
	else LED2Off();
}

void SetModuleLED2(bool led2) {
	if (led2) LED1On();
	else LED1Off();
}

void SetModuleLEDs(bool led3, bool led2) {
	SetModuleLED3(led3);
	SetModuleLED2(led2);
}

void SetRobotLEDs(uint8_t ledPColor, uint8_t ledPIntensity, bool led1, bool led0) {
	uint8_t led10 = (led1 ? LEDPlay : 0) | (led0 ? LEDAdvance : 0);
	_byteTx(CmdLeds);
	_byteTx(led10); 
	_byteTx(ledPColor);   
	_byteTx(ledPIntensity);
} 

void DefineSong(uint8_t song_number, uint8_t number_of_notes, uint8_t notes[], uint8_t durations[])
{
	_byteTx(CmdSong);
	_byteTx(song_number);
	_byteTx(number_of_notes);
	for (uint8_t i = 0; i < number_of_notes; i++) {
		_byteTx(notes[i]);
		_byteTx(durations[i]);
	}
}

int8_t PlaySong(uint8_t song_number)
{
	if (IsPlayingSong())
		return -1;
	_last_song = song_number & 0xf;
	_byteTx(CmdPlay);
	_byteTx(song_number);
	return 0;
}

void KillSong(uint8_t song_number)
{
	DefineSong(song_number, 0, NULL, NULL);
	PlaySong(song_number);
}


bool IsPlayingSong()
{
    // _refreshSensors();
	//return (_sensor_rx[SENSOR_RX_DataOISong] != 0);
    return false; // fixme
}

int8_t CurrentPlayingSong()
{
	if (IsPlayingSong())
        return _last_song;
    else
		return -1;
}

void StartMoving(int16_t velocity, int16_t radius)
{
	_byteTx(CmdDrive);
	_byteTx((uint8_t)((velocity >> 8) & 0x00FF));
	_byteTx((uint8_t)(velocity & 0x00FF));
	_byteTx((uint8_t)((radius >> 8) & 0x00FF));
	_byteTx((uint8_t)(radius & 0x00FF));
}

uint16_t PickRandomAtMost(uint16_t max_num) {
	uint16_t r = random();
	// Handle 0 and cases where max_num+1 is a power of 2.
	switch (max_num) {
		case 0xFFFF: case 0x7FFF: case 0x3FFF: case 0x1FFF:
		case 0x0FFF: case 0x07FF: case 0x03FF: case 0x01FF:
		case 0x00FF: case 0x007F: case 0x003F: case 0x001F:
		case 0x000F: case 0x0007: case 0x0003: case 0x0001:
		case 0:
			return r & max_num;
	}
	// Handle all other cases.
	// Find the smallest mask greater than max_num, then pick
	// repeatedly until we find a good value.
	uint16_t mask = 0x1;
	while (mask < max_num)
		mask = (mask << 1) | 1;
	r &= mask;
	while (r > max_num)
		r = random() & mask;
	return r;
}

int16_t PickRandomAngle() {
	uint16_t r = PickRandomAtMost(0xFF);
	int16_t r_angle = (r&7) * 45 + (r >> 3);
	r_angle -= 16;
	if (r_angle > 180)
		r_angle -= 360;
	return r_angle;
}

#ifdef ARDUINO_SERIAL_ENABLED
volatile uint8_t _arduino_data = 0;
volatile uint8_t _arduino_idx = 0;
#endif

// Timer 1 interrupt
// SIGNAL(SIG_OUTPUT_COMPARE1A)
ISR(TIMER1_COMPA_vect)
{
	if (_timer_cnt)
		_timer_cnt--;
    _timer_expired = (_timer_cnt == 0);
    if (_sensor_rx_timer)
        _sensor_rx_timer--;
    if (_telemetry_rx_timer)
        _telemetry_rx_timer--;
    _uptime_ms += TICK_INTERVAL;
	if (_uptime_ms >= 1000) {
		_uptime_ms -= 1000;
		_uptime_s++;
	}
#ifdef ARDUINO_SERIAL_ENABLED
#if TICK_INTERVAL != 3
#error When ARUINO_SERIAL_ENABLED is set to 1, TICK_INTERVAL must be set to 3.
#endif
    // This implements 8n1 serial, but at TICK_INTERVAL = 3.3 ms per bit, 
    // or 300 baud, or about 27 bytes per second max, for Arduino.
    switch (_arduino_idx) {
        case 0:
            return;
        case 1: // start bit
            PORTB &= ~0x0F;
            _arduino_idx++;
            break;
        case 2: case 3: case 4: case 5:
        case 6: case 7: case 8: case 9:
            if (_arduino_data & 0x1) {
                PORTB |= 0x0F;
            } else {
                PORTB &= ~0x0F;
            }
            _arduino_data >>= 1;
            _arduino_idx++;
            break;
        case 10: // stop bit 1
            PORTB |= 0x0F;
            _arduino_idx++;
            break;
        default: // done
            _arduino_idx++;  // delay padding
            if (_arduino_idx > 50)
                _arduino_idx = 0;
            return;
    }
#endif
}

#ifdef ARDUINO_SERIAL_ENABLED
void _ArduinoSend(uint8_t data)
{
    while (_arduino_idx)
        Delay(10);
    _arduino_data = data;
    _arduino_idx = 1;
}

void _ArduinoSendString(const char *buf)
{
    while (*buf != '\0')
        _ArduinoSend(*buf++);
}

void _ArduinoSendInt(int16_t x) // -32768 <= x <= +32767
{
    if (x == -32768) {
        _ArduinoSendString("-32768");
    } else {
        bool negative = (x < 0);
        if (negative)
            x = x * (-1);
        char buf[7]; // sign + 5 digits + nul
        uint8_t i = 7;
        buf[--i] = '\0';
        do {
            buf[--i] = '0' + (x % 10);
            x = x / 10;
        } while (x != 0);
        if (negative)
            buf[--i] = '-';
        else
            buf[--i] = '+';
        _ArduinoSendString(&buf[i]);
    }
}

void _ArduinoSendUInt(uint16_t x) // 0 <= x <= 65535
{
    char buf[6]; // 5 digits + nul
    uint8_t i = 6;
    buf[--i] = '\0';
    do {
        buf[--i] = '0' + (x % 10);
        x = x / 10;
    } while (x != 0);
    _ArduinoSendString(&buf[i]);
}

#else // if not defined ARUDINO_SERIAL_ENABLED

void _ArduinoSend(uint8_t data)
{
    /* do nothing */
}

void _ArduinoSendString(const char *buf)
{
    /* do nothing */
}

void _ArduinoSendInt(int16_t x)
{
    /* do nothing */
}

void _ArduinoSendUInt(uint16_t x)
{
    /* do nothing */
}

#endif

// Initialize the Command Module's ATmega168 microcontroller
void _initializeCommandModule() {
	cli();

	// Set I/O pins
	DDRB = 0x10;
    DDRB |= 0x0F; // cargo/left/right/center pin 3 (atmega PB0,PB1,PB2,PB3) to output mode for Arduino
	PORTB = 0xCF; // cargo/left/right/center pin 3 (atmega PB0,PB1,PB2,PB3) is high initially
	DDRC = 0x00;
	PORTC = 0xFF;
	DDRD = 0xE6;
	PORTD = 0x7D;

    // Set up ticks at TICK_INTERVAL.
    // Clock = 18432000 Hz
	TCCR1A = 0x00;
#if TICK_INTERVAL == 1
    // Set up timer 1 to generate an interrupt every 1 ms = 1000 Hz
	TCCR1B = (_BV(WGM12) | _BV(CS12)); /* ctr mode, prescale=256 */
	OCR1A = 71; // max = Clock/(prescale*1000) - 1 = 71 
#endif
#if TICK_INTERVAL == 3
    // Set up timer 1 to generate an interrupt every 3.3 ms = 300 Hz
	TCCR1B = (_BV(WGM12) | _BV(CS12) | _BV(CS10)); /* ctr mode, prescale=1024 */
	OCR1A = 59;  // max = Clock/(prescale*300) - 1 = 59;
#endif
#if TICK_INTERVAL == 10
    // Set up timer 1 to generate an interrupt every 10 ms = 100 Hz
	TCCR1B = (_BV(WGM12) | _BV(CS12) | _BV(CS10)); /* ctr mode, prescale=1024 */
	OCR1A = 179;  // max = Clock/(prescale*100) - 1 = 179
#endif
	TIMSK1 = _BV(OCIE1A);

	// Set up the serial port with rx interrupt if needed
	UBRR0 = Ubrr57600;
	// UCSR0B = (_BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0)); /* rx complete interrupt enable, tx enable, rx enable */
	UCSR0B = (_BV(TXEN0) | _BV(RXEN0)); /* tx enable, rx enable */
	UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01)); /* 8N1 */

	// Turn on interrupts
	sei();
}


void _powerOnRobot() {
	// If Create's power is off, turn it on
	if (!RobotIsOn()) {
		while (!RobotIsOn()) {
			RobotPwrToggleLow();
			Delay(500);  // Delay in this state
			RobotPwrToggleHigh();  // Low to high transition to toggle power
			Delay(100);  // Delay in this state
			RobotPwrToggleLow();
		}
		Delay(3500);  // Delay for startup
	}
}


// Switch the baud rate on both Create and module
void _setSerialSpeed(uint8_t baud) {
	if (baud <= Baud115200) {
		_byteTx(CmdBaud);
		UCSR0A |= _BV(TXC0);
		_byteTx(baud);
		// Wait until transmit is complete
		while (!(UCSR0A & _BV(TXC0)))
			;

		cli();

		switch(baud) {
			case Baud115200: UBRR0 = Ubrr115200; break;
			case Baud57600: UBRR0 = Ubrr57600; break;
			case Baud38400: UBRR0 = Ubrr38400; break;
			case Baud28800: UBRR0 = Ubrr28800; break;
			case Baud19200: UBRR0 = Ubrr19200; break;
			case Baud14400: UBRR0 = Ubrr14400; break;
			case Baud9600: UBRR0 = Ubrr9600; break;
			case Baud4800: UBRR0 = Ubrr4800; break;
			case Baud2400: UBRR0 = Ubrr2400; break;
			case Baud1200: UBRR0 = Ubrr1200; break;
			case Baud600: UBRR0 = Ubrr600; break;
			case Baud300: UBRR0 = Ubrr300; break;
		}

		sei();

		Delay(150); // was 100
	}
}

// Transmit a byte over the serial port
void _byteTx(uint8_t value) {
	while (!(UCSR0A & _BV(UDRE0)))
		;
	UDR0 = value;
}
