//
//  Log ROS connection
//
//  Created by Andrei Cristian Popescu on 07/05/2021.
//

#include <Log_ROS.hpp>

void Log_ROS::init(){
    if(initialised)
        return;

    nh.initNode();
    initialised = true;
}

Log_ROS::Log_ROS(){
    init();
}

void Log_ROS::Log(const String &msg, const severity_type &severity){
    if(!isConnected())
        Serial.println("Logger: Unable to connect to ROS!");

    String message = msg;

    switch(severity){
        case debug:
            nh.logdebug(message.c_str());
            break;
        case warning:
            nh.logwarn(message.c_str());
            break;
        case error:
            nh.logerror(message.c_str());
            break;
    }
    
    nh.spinOnce();
}

bool Log_ROS::isConnected(){
    return nh.connected();
}