#include "interfacetest.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899


InterfaceTest::InterfaceTest()
{
}

void InterfaceTest::test()
{
    ServiceInterface robotService;

    aubo_robot_namespace::SeamTracking seamTrack;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin("", SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录失败."<<std::endl;
    }
    else
    {
        std::cerr<<"登录成功."<<std::endl;
    }

    seamTrack.trackEnable = true;
    seamTrack.timeInterval = 123;
    seamTrack.currentPosError[0] = 1.1;
    seamTrack.currentPosError[1] = 2.2;
    seamTrack.currentPosError[2] = 3.3;

    seamTrack.maxVel = 1.234;
    seamTrack.maxAcc = 3.123;
    seamTrack.paraChanged = false;

    for(int i=0;i<6;i++)
    {
        seamTrack.currentRoadPoint.jointpos[i] = i+i*0.1;
        seamTrack.nextRoadPoint.jointpos[i] = i*10+i*0.1;
    }

    robotService.robotServiceSetSeamTrackingParameters(seamTrack);

    sleep(10);
}
