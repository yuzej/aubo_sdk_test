#include "example_toolio.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "AuboRobotMetaType.h"    //Robot arm metadata type
#include "serviceinterface.h"     //Robot arm interface

#define SERVER_HOST "127.0.0.1"
//#define SERVER_HOST "192.168.1.123"
#define SERVER_PORT 8899



void Example_ToolIO::demo()
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

    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    robotService.rootServiceRobotStartup(toolDynamicsParam, 6, true, true, 1000, result, true);    //Robot arm initialization


    /** Set the power of the tool end IO and the type of digital IO **/
    ret = robotService.robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::OUT_12V,
                                                              aubo_robot_namespace::IO_OUT, aubo_robot_namespace::IO_IN,
                                                              aubo_robot_namespace::IO_IN, aubo_robot_namespace::IO_OUT);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"Set the tool end IO type SUCC."<<std::endl;
    }
    else
    {
        std::cerr<<"Set the tool end IO type Failed."<<std::endl;
    }

    sleep(2);   //Delay
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;
    ret = robotService.robotServiceGetAllToolDigitalIOStatus(statusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"Digital IO Count:"<<statusVector.size()<<std::endl;

        for(int i=0;i<(int)statusVector.size();i++)
        {
            std::cout<<"ioAddr:"<<statusVector[i].ioAddr<<"  ioType:"<<statusVector[i].ioType<<"  ioName:"<<statusVector[i].ioName<<std::endl;
        }
    }
    else
    {
        std::cerr<<"Set the tool end IO type Failed."<<std::endl;
    }
}
