#include <iostream>

#include <stdio.h>
#include <unistd.h>

#include "example_0.h"
#include "example_1.h"
#include "example_3.h"
#include "example_4.h"
#include "example_5.h"
#include "example_6.h"
#include "example_8.h"
#include "example_9.h"


int main()
{
    //Case 0: Using the SDK to build a control engineering for the simplest robotic arm
//    Example_0::demo();

    //Case 1: The way of the callback function to obtain the real-time waypoint,
    //the end speed, the event of the manipulator, about the acquisition of the relevant state of the manipulator
//    Example_1::demo();
//    Example_1::getJointStatus();

    //Case 3: Joint Movement
    Example_3::demo();

    //Case 4: Line Movement
//   Example_4::demo();
//   Example_4::demo_relativeOri();

    //Case 5: Trajectory movement
//    Example_5::demo1();

    //Case 6: Movement to the target position
//    Example_6::demo();

    //Case 8: Positive and negative solutions
//    Example_8::demo();

    //Case 9: About the use case of io
//    Example_9::demo();

    return 0;
}

