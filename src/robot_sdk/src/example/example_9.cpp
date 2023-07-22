#include "example_9.h"


#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#include <vector>

#include "AuboRobotMetaType.h"    //Robot arm metadata type
#include "serviceinterface.h"     //Robot arm interface


//#define SERVER_HOST "127.0.0.1"
#define SERVER_HOST "192.168.80.69"
#define SERVER_PORT 8899



void Example_9::demo()
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

    //Get digital IO status
    double value;
    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, "U_DI_01", value);
    std::cout<<"U_DI_01 status:"<<value<<std::endl;

    std::vector<aubo_robot_namespace::RobotIoType> ioType;
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

    ioType.push_back(aubo_robot_namespace::RobotBoardUserAI);
    robotService.robotServiceGetBoardIOStatus(ioType, statusVector);

//    for(int i=0;i<statusVector.size();i++)
//    {
//        std::cout<<statusVector[i].ioName<<"   "<<statusVector[i].ioValue<<std::endl;
//    }



    //Get the analog IO status
    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAI, "VI0", value);
    std::cerr<<"---------VI0 status:"<<value<<std::endl;

    //Set the analog IO status
    ret = robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_02", 0);
    std::cerr<<"ret:"<<ret<<std::endl;

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}



