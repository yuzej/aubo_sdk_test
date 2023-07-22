#include "example_0.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

void Example_0::demo()
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


    /** If the real robot arm is connected, the arm needs to be initialized.　**/
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

    /** Simulation business **/
    std::cout<<"Simulation business..."<<std::endl;
    sleep(5);

    /** Robotic arm shutdown **/
    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();
}
