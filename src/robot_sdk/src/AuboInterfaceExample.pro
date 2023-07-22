TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt


unix{
    #32bit OS
    contains(QT_ARCH, i386){

        CONFIG += c++11

        DEFINES += _GLIBCXX_USE_CXX11_ABI=0

        INCLUDEPATH += $$PWD/dependents/robotSDK/inc

        LIBS += -L$$PWD/dependents/robotSDK/lib/linux_x32/ -lauborobotcontroller

        LIBS += -lpthread

    }
    #64bit OS
    contains(QT_ARCH, x86_64){

        INCLUDEPATH += $$PWD/dependents/robotSDK/inc

        LIBS += -L$$PWD/dependents/log4cplus/linux_x64/lib/ -llog4cplus

        LIBS += -L$$PWD/dependents/robotSDK/lib/linux_x64/ -lauborobotcontroller

        LIBS += -lpthread
    }
}


#功能示例
include(./example/example.pri)
#include(./applicationPoint/application_point.pri)

#接口测试
#include(./interfaceTest/interfacetest.pri)

#调试:SDK源码子工程
#include(/home/carl/AuboWorkspace/development/source/OUR-I5/trunk/sdk/RobotInterfaceDevelop/sdk-source/sdk.pri)

SOURCES += \
    main.cpp


