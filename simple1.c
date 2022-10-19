/*******************************************************************************
 * simple1.c
 * A very simple iRobot program to demo in lab.
 * Demonstrates the use of a conditional.
 * Written by kwalsh@cs.holycross.edu.
 *******************************************************************************/

#include "iRobot.h"

int main(void)
{
	WakeRobot();

	bool ok;

    ok = Straight(500);

	if (!ok) {
        // Bumped into something. Turn right.
        Turn(-90);
    } else {
        // No bumps. Turn left.
		Turn(90);
    }

	Straight(500);
}

