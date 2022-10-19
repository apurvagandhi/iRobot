/*******************************************************************************
 * simple.c
 * A very simple iRobot program to demo in lab.
 * Written by kwalsh@cs.holycross.edu.
 *******************************************************************************/

#include "iRobot.h"

int main(void)
{
	WakeRobot();

	Turn(90);
	Straight(1000);
	WaitForBlackButton();
	Turn(360);
	Straight(1000);
}

