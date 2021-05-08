//
//  A logger to ROS a topic
//
//  Created by Andrei Cristian Popescu on 07/05/2021.
//

#include <ROS_Logger.hpp>

/// The message that is gonna be sent to ROS
std_msgs::String ros_message;

/// The main communication channel to the ROS
ros::Publisher logger("logger", &ros_message);

ROS_Logger::ROS_Logger(){
    nh.initNode();
    nh.advertise(logger);
}

void ROS_Logger::Log(const String &msg){
    if(!isConnected())
        Serial.println("Logger: Unable to connect to ROS!");

    String message = msg;
    ros_message.data = message.c_str();
    
    logger.publish(&ros_message);
    
    nh.spinOnce();
}

bool ROS_Logger::isConnected(){
    return nh.connected();
}