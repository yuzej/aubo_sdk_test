#include "example_3.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"

#define SERVER_HOST "192.168.1.40"
//#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

void Example_3::demo()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** Interface call: login ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"login successful."<<std::endl;
    }
    else
    {
        std::cerr<<"login failed."<<std::endl;
    }

     std::cout<<"Robot arm initialization....."<<std::endl;


     /** If the real robot arm is connected, the arm needs to be initialized.**/
     aubo_robot_namespace::ROBOT_SERVICE_STATE result;

     //Tool dynamics parameter
     aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
     memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

     ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**Tool dynamics parameter**/,
                                                6        /*Collision level*/,
                                                true     /*Whether to allow reading poses defaults to true*/,
                                                true,    /*Leave the default to true */
                                                1000,    /*Leave the default to 1000 */
                                                result); /*Robot arm initialization*/
     if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
     {
         std::cout<<"Robot arm initialization succeeded."<<std::endl;
     }
     else
     {
         std::cerr<<"Robot arm initialization failed."<<std::endl;
     }

    /** Business block **/
    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** Interface call: set the maximum speed of articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** Robot arm movement to zero posture **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    std::cout<<"Calling the motion function"<<std::endl;
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Movement to zero posture failure.　ret:"<<ret<<std::endl;
    }

    for(int i=0;i>-1;i++)
    {
//        /** Interface call: set offset**/
//        aubo_robot_namespace::MoveRelative relativeMoveOnBase;
//        relativeMoveOnBase.ena = true;
//        relativeMoveOnBase.relativePosition[0] = 0;
//        relativeMoveOnBase.relativePosition[1] = 0;
//        relativeMoveOnBase.relativePosition[2] = 0.05*(i%4);   //Unit: m

//        relativeMoveOnBase.relativeOri.w=1;
//        relativeMoveOnBase.relativeOri.x=0;
//        relativeMoveOnBase.relativeOri.y=0;
//        relativeMoveOnBase.relativeOri.z=0;
//        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);


        /** Waypoint 1 movement **/
        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 1 failed.　ret:"<<ret<<std::endl;
            break;
        }

        /** Waypoint 2 movement **/
        Util::initJointAngleArray(jointAngle, 15.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 2 failed.　ret:"<<ret<<std::endl;
            break;
        }
    }

    sleep(10);

    /** Robotic arm shutdown **/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}
