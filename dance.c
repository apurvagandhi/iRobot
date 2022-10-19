/*******************************************************************************
 shape.c
 *Apurva Gandhi, Yusi Yao, Brenden Sheedy, Daniel Gilbert 
 *ahgand22@g.holycross.edu
 *Written: 10/15/2018
 *Rewritten: 10/17/2018
 *A iRobot program to clear the maze by making an repatitive turns
 *based on the distance along with different music
 *and lights according to the turn angle.  
 *******************************************************************************/
#include "iRobot.h"

int main(void)
{
    // start the body of the robot
    WakeRobot();
    ResetOdometer();
    //Music when it finish the maze
    uint8_t notes0[] = {60, 60, 66, 66, 68, 68, 66, 64, 64, 63, 63, 62, 62, 60, 70, 71};
    uint8_t durations0[] = {16, 16, 16, 16, 16, 16, 32, 16, 16, 16, 16, 16, 16, 32, 16, 16};
    DefineSong(0,16, notes0,durations0);

    //Music when it hits the 90 degree turn
    uint8_t notes1[] = {67, 57};
    uint8_t durations1[] = {16,16};
    DefineSong(1,2, notes1,durations1);

    //Music when it hits the 180 degree turn
    uint8_t notes2[] = {54, 33, 35, 35, 35, 33, 54, 54, 54, 33, 35, 35, 35, 33, 54, 28};
    uint8_t durations2[] =  {16,16,16,16,16,16,16,32,16,16,16,16,16,16,16,16};
    DefineSong(2,16, notes2,durations2);
    
    //Setting the  LED's
    bool led3 = true;
    bool led2 = true;
    bool led1 = false;
    bool led0 = false;
    SetRobotLEDs(255, 0, led1, led0);
    //Strts walking 3 meters
    bool walking  = Forward(3000);
    //If bumpers are not pressed
    if(OdometerDistance()>2990)
    {
        PlaySong(0);
        Stop();
    }
    //if any bumpers are pressed 
    else
    {
        //Wait for bumping to occur
        //It will stop when robot will travels 3 meter straight
        while(OdometerDistance() < 3000)
        {
            //If the bumper is pressed, turn 93 degree to right
            if (walking == false)
            {
                //Play song 1
                PlaySong(1);
                led1 = true;
                led0 = true;
                //Turn on the small LED's
                SetRobotLEDs(255, 255, led1, led0);
                //Turn the robot
                Turn(-93);
                ResetOdometer();
                //Turn off the small LED's
                led1 = false;
                led0 = false;
                SetRobotLEDs(255, 0, led1, led0);
            }
            walking = Forward(3000);
            //If the bumpers are pressed within half meter distance
            if( walking == false && OdometerDistance() < 500)
            {
                //Play song 
                PlaySong(2);
                //Turn on the LED's
                led1 = true;
                led0 = true;
                led2 = true;
                led3 = true;
                SetRobotLEDs(0, 255, led1, led0);
                SetModuleLEDs(led3, led2);
                //Turn 180 degree to left
                Turn(180);
                ResetOdometer();
                //Turn off the LED's
                led0 = false; 
                led1 = false;
                led2 = false;
                led3 = false;
                SetRobotLEDs(0, 0, led1, led0);
                SetModuleLEDs(led3, led2);
                //Walking straight for 3 meter
                walking = Forward(3000);              
            }//end of if condition
        }// end of while
    }// end of else
    //Checks if robot has travelled 3 meters and stops if it is true
    if(OdometerDistance()>2990)
    {
        PlaySong(0);
        Stop();
    }
}//end of main
