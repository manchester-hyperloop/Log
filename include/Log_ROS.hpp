//
//  Log ROS connection
//
//  Created by Andrei Cristian Popescu on 07/05/2021.
//

#include <Severity_Type.hpp>

#include <ros.h>
#include <std_msgs/String.h>

/**
 * The logger ROS connection.
 */

class Log_ROS{
private:
    /// The main access point that communicates
	/// with the ROS.
	ros::NodeHandle nh;

    bool initialised = false;

    /**
     * Initialise the ROS connection.
     */
    void init();    

public:
    /**
     * The class constructor:
     * Initialise the ROS connection.
    */
    Log_ROS();

    /**
     * Log a message to ROS.
     * @param msg: The message that is gonna be logged to ROS.
     * @param severity: The severity type of the logged message.
    */
    void Log(const String &msg, const severity_type &severity);

    /**
     * Check if the logger is connected to ROS .
     * @return bool - If the class is connected to
     * ROS or not.
    */
    bool isConnected();
};