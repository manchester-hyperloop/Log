//
//  A logger to ROS a topic
//
//  Created by Andrei Cristian Popescu on 07/05/2021.
//

#include <Severity_Type.hpp>

#include <ros.h>
#include <std_msgs/String.h>

/**
 * The logger ROS connection.
 */

class ROS_Logger{
private:
    /// The main access point that communicates
	/// with the ROS.
	ros::NodeHandle nh;

public:
    /**
     * The class constructor:
     * Initialise the ROS connection.
    */
    ROS_Logger();

    /**
     * Log a message to ROS.
     * @param msg: The message that is gonna be logged to ROS.
    */
    void Log(const String &msg);

    /**
     * Check if the logger is connected to ROS .
     * @return bool - If the class is connected to
     * ROS or not.
    */
    bool isConnected();
};