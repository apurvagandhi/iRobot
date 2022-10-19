/*******************************************************************************
 * random.c
 * A simple iRobot program to use in lab.
 * Plays a few songs while crashing into things.
 *
 * Original from iRobot Create Command Module source code.
 * Modified by Christina D'Ambrogio, June 2008.
 * Re-written March 2009 to use robotAPI.c, L.King.
 * Re-written Sept 2012 to use iRobot.h, K. Walsh.
 *
 * Program Explanation:
 * Plays the song, Star Wars Imperial March.
 * Drives robot until a bump is detected.
 * Rotates at a random angle.
 * Repeats the above steps until the black button is pressed or an
 * unsafe condition is detected.
 * Before exiting, the program plays the Iron Man song.
 * 
 *******************************************************************************/

// Include the new iRobot API
#include "iRobot.h"

// Notes for Reset Song
// C, C, C, C, C, C, C, C
uint8_t reset_notes[] = {60, 72, 84, 96, 60, 72, 84, 96};
uint8_t reset_times[] = { 6,  6,  6,  6,  6,  6,  6,  6};
uint8_t reset_count = 8; // 8 notes in this song

// Notes for Star Wars Imperial March
// G, G, G, D#, A#, G, D#, A#, G, D,
// D, D, D#, A#, F#, D#, A#, G
uint8_t starwars_notes[] =
{55, 55, 55, 51, 58, 55, 51, 58, 55, 62, 62, 62, 63, 58, 54, 51, 58, 55};
uint8_t starwars_times[] =
{32, 32, 32, 16, 16, 32, 16, 16, 32, 32, 32, 32, 16, 16, 32, 16, 16, 32};
uint8_t starwars_count = 18; // 18 notes in this song

// Notes for Bump Song
// C, C
uint8_t bump_notes[] = {72, 72};   
uint8_t bump_times[] = {32, 32};   
uint8_t bump_count = 2; // 2 notes in this song

// Notes for Iron Man Song
// A, C, C, D, D, F, E, F, E, F,
// E, C, C, D, D
uint8_t ironman_notes[] = {57, 60, 60, 62, 62, 65, 64, 65, 64, 65, 64, 60, 60, 62, 62};
uint8_t ironman_times[] = {90, 60, 30, 30, 60, 15, 15, 15, 15, 15, 20, 30, 30, 30, 30};
uint8_t ironman_count = 15; // 15 notes in this song

// Give each song a slot number. 
const int RESET_SONG = 0;
const int STAR_WARS_IMPERIAL_MARCH_SONG = 1;
// Note: Slot 2 is taken up by Star Wars since that song has more than 16 notes.
const int BUMP_SONG = 3;
const int IRON_MAN_SONG = 4;

int main (void)
{

    WakeRobot(); // this powers up the iRobot body if needed

    DefineSong(RESET_SONG, reset_count, reset_notes, reset_times);
    DefineSong(STAR_WARS_IMPERIAL_MARCH_SONG, starwars_count, starwars_notes, starwars_times);
    DefineSong(BUMP_SONG, bump_count, bump_notes, bump_times);
    DefineSong(IRON_MAN_SONG, ironman_count, ironman_notes, ironman_times);

    // Play the reset song.
    PlaySong(RESET_SONG);

    // Set the power LED to bright orange, and turn on the other two iRobot LEDs.
    SetRobotLEDs(ORANGE, BRIGHT, true, true);

    // Pause until someone presses the black button to start the random walk.
    WaitForBlackButton();

    SetRobotLEDs(GREEN, BRIGHT, true, true);

    // Play a "frightening" song
    PlaySong(STAR_WARS_IMPERIAL_MARCH_SONG);

    // Do nothing for for 7 seconds (i.e. 7000 milliseconds). This should
    // be about enough time for the Star Wars song to finish playing.
    Delay(7 * 1000);

    // Now repeatedly wander around.
    // Start by driving straight somewhere (this is like a priming read for the while-loop).
    bool ok;
    ok = Straight(2 * METER);

    // Now loop so long as we keep bumping 
    while (!ok)
    {
        // Play a song and turn a random amount
        SetRobotLEDs(RED, BRIGHT, true, true);
        PlaySong(BUMP_SONG);
        int random_angle = PickRandomBackwardsAngle();
        Turn(random_angle);

        // Now walk again
        SetRobotLEDs(GREEN, BRIGHT, true, true);
        ok = Straight(2 * METER);
    } // end while loop

    // If the program reaches here, iRobot must have finished its walk without
    // bumping anything.

    // Wait a second to let any previous song finish playing.
    Delay(1 * SECOND);

    // Then play one final song
    PlaySong(IRON_MAN_SONG);

    SetRobotLEDs(ORANGE, BRIGHT, false, false);

    // Wait a few second to let the song finish
    Delay(10*SECOND);

    return 0;
} // end of main
