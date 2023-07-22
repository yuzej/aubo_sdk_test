#include "example_4.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#include "util.h"


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

aubo_robot_namespace::wayPoint_S Example_4::s_currentWayPoing;


void Example_4::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    s_currentWayPoing = *wayPointPtr;
    //Util::printWaypoint(s_currentWayPoing);
}


void Example_4::demo()
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


    robotService.robotServiceRegisterRealTimeRoadPointCallback(Example_4::RealTimeWaypointCallback, NULL);


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

   /** Move to initial pose **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Movement to zero posture failure.　ret:"<<ret<<std::endl;
    }


    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the end type motion Linear motion belongs to the end type motion***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 4;   //Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** Interface call: Set the maximum speed of the end type motion Linear motion belongs to the end type motion***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 4;   //Units m/s
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    for(int i=0;i<20;i++)
    {
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        ret = robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 0 failed.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 1 failed.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 30.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 2 failed.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 30.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 3 failed.　ret:"<<ret<<std::endl;
        }
    }


    /** Robotic arm shutdown**/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}



void Example_4::demo_relativeOri()
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

    robotService.robotServiceRegisterRealTimeRoadPointCallback(Example_4::RealTimeWaypointCallback, NULL);

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

   /** Move to initial pose **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Movement to zero posture failure.　ret:"<<ret<<std::endl;
    }


    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the end type motion Linear motion belongs to the end type motion***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 1.0;   //Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** Interface call: Set the maximum speed of the end type motion Linear motion belongs to the end type motion***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 2.0;   //Units m/s
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    //Define a tool
    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x=0;
    toolInEndDesc.toolInEndPosition.y=0;
    toolInEndDesc.toolInEndPosition.z=0.1;
    toolInEndDesc.toolInEndOrientation.w=1;
    toolInEndDesc.toolInEndOrientation.x=0;
    toolInEndDesc.toolInEndOrientation.y=0;
    toolInEndDesc.toolInEndOrientation.z=0;

    for(int i=0;i<30;i++)
    {
        aubo_robot_namespace::Rpy relativeRpy;
        relativeRpy.rx=1.0/180.0*M_PI;
        relativeRpy.ry=1.0/180.0*M_PI;
        relativeRpy.rz=0.0/180.0*M_PI;

        aubo_robot_namespace::Ori relativeOri;
        robotService.RPYToQuaternion(relativeRpy,relativeOri);  //Euler angle to quaternion to find the four-element representation of the offset

        aubo_robot_namespace::MoveRelative relativeMoveOnUserCoord;
        relativeMoveOnUserCoord.ena = true;
        relativeMoveOnUserCoord.relativePosition[0] = 0;
        relativeMoveOnUserCoord.relativePosition[1] = 0;
        relativeMoveOnUserCoord.relativePosition[2] = 0;
        relativeMoveOnUserCoord.relativeOri = relativeOri;

        aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
        userCoord.coordType=aubo_robot_namespace::EndCoordinate;
        userCoord.toolDesc=toolInEndDesc;

        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnUserCoord, userCoord);   //Set offset
        robotService.robotServiceSetToolKinematicsParam(toolInEndDesc);

        robotService.robotServiceLineMove(s_currentWayPoing.jointpos,true);
    }


    /** Robotic arm shutdown**/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}
