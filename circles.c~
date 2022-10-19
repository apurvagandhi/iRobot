/*******************************************************************************
 * circles.c
 * A simple iRobot program to demo in lab.
 * Drives in ever-widening circles, up to the max turning radius.
 * Re-written by kwalsh@cs.holycross.edu, based on original simple.c.
 *******************************************************************************/
#include "iRobot.h"

// Notes for button song
uint8_t button_notes[] = {65, 65, 71};
uint8_t button_times[] = {12, 8, 32};
uint8_t button_count = 3; // three notes

// Notes for bump song
uint8_t bump_notes[] = {71, 67}; // B, G
uint8_t bump_times[] = {24,  8}; // three-eighth, one-eighth
uint8_t bump_count = 2; // two notes

int main(void)
{
	bool ok;

	WakeRobot();

	// define some songs
	DefineSong(0, button_count, button_notes, button_times);
	DefineSong(1, bump_count, bump_notes, bump_times);

	WaitForBlackButton();
	PlaySong(0);

	// spin around
	ok = TurnOrBump(360);

	// if no problems, make a larger circle, 1ft radius
	if (ok)
		ok = DriveOrBump(WALK, 1*FOOT, 0, 360);

	// if no problems, make a larger circle, 3ft radius
	if (ok)
		ok = DriveOrBump(WALK, 3*FOOT, 0, 360);

	// if no problems, make a larger circle, 5ft radius
	if (ok)
		ok = DriveOrBump(WALK, 5*FOOT, 0, 360);

	// max turning radius is 2 meters, so now just finish with
	// a song if we bumped something.
	if (!ok) {
		Straight(-6 * INCH);
		Turn(360);
		PlaySong(1);
	}

}

