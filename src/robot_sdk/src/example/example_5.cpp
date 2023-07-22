#include "example_5.h"

#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"



#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899


void Example_5::demo()
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
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   ////The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** Interface call: set the maximum speed of articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   ////The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Robot arm movement to zero posture **/
    double endMoveMaxAcc;
    endMoveMaxAcc = 0.2;   //Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);


    /** Interface call: Set the maximum speed of the end type motion Linear motion belongs to the end type motion***/
    double endMoveMaxVelc;
    endMoveMaxVelc = 0.2;   //Units m/s
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);

    double jointAngle[aubo_robot_namespace::ARM_DOF];

    for(int i=0;i<1;i++)
    {
//        //Preparation point joint movement is joint movement
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //Joint movement to preparation point
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"MoveJoint failed.　ret:"<<ret<<std::endl;
//        }


//        //Arc
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        Util::initJointAngleArray(jointAngle,0.200000, -0.127267, -1.321122, 0.376934, -1.570794, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        Util::initJointAngleArray(jointAngle,0.600000, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        robotService.robotServiceSetGlobalCircularLoopTimes(0);    //Circle number
//        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);

//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//        }


//        //Preparation point
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //Joint movement to preparation point
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"Movevfailed.　ret:"<<ret<<std::endl;
//        }

//        //circle
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);
//        Util::initJointAngleArray(jointAngle,-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);
//        Util::initJointAngleArray(jointAngle,-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        robotService.robotServiceSetGlobalCircularLoopTimes(1);    //Circle number
//        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR,true);
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//        }


        //Preparation point
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //Joint movement to preparation point
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"MoveJoint failed.　ret:"<<ret<<std::endl;
//        }

        //MoveP
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        Util::initJointAngleArray(jointAngle,0.100000, -0.147267, -1.321122, 0.376934, -1.570794, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        Util::initJointAngleArray(jointAngle,0.200000, -0.167267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        robotService.robotServiceSetGlobalBlendRadius(0.03);                     //Blending radius
//        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_GNUBSPLINEINTP,true);
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//        }
    }

    /** Robotic arm shutdown**/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}



void Example_5::demo1()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    //Login
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"login successful."<<std::endl;
    }
    else
    {
        std::cerr<<"login failed."<<std::endl;
    }

    //Initialize motion properties
    robotService.robotServiceInitGlobalMoveProfile();

    //Set the maximum acceleration of articulated motion
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i=0;i<6;i++)
    {
        jointMaxAcc.jointPara[i]  = 20.0/180.0*M_PI;   //The unit is radians
        jointMaxVelc.jointPara[i] = 20.0/180.0*M_PI;   //The unit is radians
    }
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    //Set the maximum acceleration of the end motion. Linear motion is the end motion.
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(2);    //Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(2);   //Units m/s


    //Preparation point
    double jointAngle[aubo_robot_namespace::ARM_DOF];
    Util::initJointAngleArray(jointAngle, 0.0, 0.0, 90.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0);
    ret = robotService.robotServiceJointMove(jointAngle, true);   //Joint movement to preparation point
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Joint move failed.　ret:"<<ret<<std::endl;
    }


    for(int i=0;i>-1;)
    {
        //Linear motion
        robotService.robotServiceInitGlobalMoveProfile();
        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(1.0);    //Units m/s2
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(0.2);   //Units m/s

        aubo_robot_namespace::MoveRelative relativeMoveOnBase;
        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = 0;
        relativeMoveOnBase.relativePosition[1] = -0.162;   //Relative center Y-axis offset
        relativeMoveOnBase.relativePosition[2] = 0;
        relativeMoveOnBase.relativeOri.w=1;
        relativeMoveOnBase.relativeOri.x=0;
        relativeMoveOnBase.relativeOri.y=0;
        relativeMoveOnBase.relativeOri.z=0;
        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);

        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 90.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0);
        ret = robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Move failed.　ret:"<<ret<<std::endl;
            exit(1);
        }


        //First point of the arc
        aubo_robot_namespace::wayPoint_S arcStartWayPoint;    //Arc starting point
        aubo_robot_namespace::wayPoint_S arcWayPoint2;        //The second point of the arc
        aubo_robot_namespace::wayPoint_S arcWayPoint3;        //The third point of the arc

        robotService.robotServiceInitGlobalMoveProfile();
        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(1.0);    //Units m/s2
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(0.2);   //Units m/s

        robotService.robotServiceGetCurrentWaypointInfo(arcStartWayPoint);       //Get the real-time waypoint of the robot arm
        robotService.robotServiceAddGlobalWayPoint(arcStartWayPoint.jointpos);   //Add the first point to the waypoint container on the arc

        //a point on the arc through the inverse solution (the second point of the arc)
        aubo_robot_namespace::Pos position;
        position.x = arcStartWayPoint.cartPos.position.x+0.162;
        position.y = arcStartWayPoint.cartPos.position.y+0.162;
        position.z = arcStartWayPoint.cartPos.position.z;
        if(robotService.robotServiceRobotIk(arcStartWayPoint.jointpos, position, arcStartWayPoint.orientation, arcWayPoint2)!= aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"IK failed"<<std::endl;
        }
        robotService.robotServiceAddGlobalWayPoint(arcWayPoint2);  //Add the second point to the waypoint container on the arc

        //a point on the arc through the inverse solution (the third point of the arc)
        position.x = arcStartWayPoint.cartPos.position.x;
        position.y = arcStartWayPoint.cartPos.position.y+0.162*2;
        position.z = arcStartWayPoint.cartPos.position.z;
        if(robotService.robotServiceRobotIk(arcStartWayPoint.jointpos, position, arcStartWayPoint.orientation, arcWayPoint3)!= aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"IK failed"<<std::endl;
        }
        robotService.robotServiceAddGlobalWayPoint(arcWayPoint3);  //Add the third point to the waypoint container on the arc

        //Circular motion
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
            exit(1);
        }


        //Linear motion
        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = -0.162;
        relativeMoveOnBase.relativePosition[1] = 0;
        relativeMoveOnBase.relativePosition[2] = 0;   //Units m/s
        relativeMoveOnBase.relativeOri.w=1;
        relativeMoveOnBase.relativeOri.x=0;
        relativeMoveOnBase.relativeOri.y=0;
        relativeMoveOnBase.relativeOri.z=0;
        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);

        aubo_robot_namespace::wayPoint_S centerOfMindWayPoint;
        robotService.robotServiceGetCurrentWaypointInfo(centerOfMindWayPoint);
        ret = robotService.robotServiceLineMove(centerOfMindWayPoint.jointpos, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Move failed.　ret:"<<ret<<std::endl;

            exit(1);
        }


        //Setting properties
        robotService.robotServiceInitGlobalMoveProfile();
        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(1.0);    //Units m/s2
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(0.2);   //Units m/s

        //First point of the arc
        robotService.robotServiceGetCurrentWaypointInfo(arcStartWayPoint);         //Get the real-time waypoint of the robot arm
        robotService.robotServiceAddGlobalWayPoint(arcStartWayPoint.jointpos);     //Add the first point to the waypoint container on the arc

        //a point on the arc through the inverse solution (the second point of the arc)
        position.x = arcStartWayPoint.cartPos.position.x-0.162;
        position.y = arcStartWayPoint.cartPos.position.y-0.162;
        position.z = arcStartWayPoint.cartPos.position.z;
        if(robotService.robotServiceRobotIk(arcStartWayPoint.jointpos, position, arcStartWayPoint.orientation, arcWayPoint2)!= aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"IK failed"<<std::endl;
        }
        robotService.robotServiceAddGlobalWayPoint(arcWayPoint2);  //Add the second point to the waypoint container on the arc

        //a point on the arc through the inverse solution (the third point of the arc)
        position.x = arcStartWayPoint.cartPos.position.x;
        position.y = arcStartWayPoint.cartPos.position.y-0.162*2;
        position.z = arcStartWayPoint.cartPos.position.z;
        if(robotService.robotServiceRobotIk(arcStartWayPoint.jointpos, position, arcStartWayPoint.orientation, arcWayPoint3)!= aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"IK failed"<<std::endl;
        }
        robotService.robotServiceAddGlobalWayPoint(arcWayPoint3);  //Add the third point to the waypoint container on the arc

        //Circular motion
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;

            exit(1);
        }


        //Linear motion
        robotService.robotServiceInitGlobalMoveProfile();
        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(2);    //Units m/s2
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(2);   //Units m/s

        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = 0.162;
        relativeMoveOnBase.relativePosition[1] = 0;
        relativeMoveOnBase.relativePosition[2] = 0;   //Units m/s
        relativeMoveOnBase.relativeOri.w=1;
        relativeMoveOnBase.relativeOri.x=0;
        relativeMoveOnBase.relativeOri.y=0;
        relativeMoveOnBase.relativeOri.z=0;
        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);

        robotService.robotServiceGetCurrentWaypointInfo(centerOfMindWayPoint);
        ret = robotService.robotServiceLineMove(centerOfMindWayPoint.jointpos, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Move failed.　ret:"<<ret<<std::endl;

            exit(1);
        }

    }
}
