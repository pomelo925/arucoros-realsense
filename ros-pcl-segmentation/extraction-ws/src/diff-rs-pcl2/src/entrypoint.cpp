/**
 *
 * @file entrypoint.cpp
 * @brief
 *  Upon receiving the ready signal, it calls a service to initiate a specific action based on the frame ID of the received signal. 
 *  
 * Functionalities :
 * - Subscribing to the ready signal topic
 * - Defining a callback function to process the received signal
 * - Calling a service to initiate an action based on the frame ID
 * 
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author pomelo925 (yoseph.huang@gmail.com)
 * @version 2.1
 * @date 2024-03-27
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "std_srvs/SetBool.h" 

ros::ServiceClient client;


// Callback for the topic subscriber
void readySignalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  std_srvs::SetBool srv;
  srv.request.data = true; 

  // Check frame_id to see if we should proceed with the service ca
  if (msg->header.frame_id == "inspection") {
    if (client.call(srv)) system("/home/startup/diff/diff-inspection.sh");
    else ROS_ERROR("Failed to call service.");
  }
  
  else {
    srv.request.data = false; 
    if (!client.call(srv)) ROS_ERROR("Failed to call service.");
  }

  ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ready_signal_processor");
    ros::NodeHandle nh;

    client = nh.serviceClient<std_srvs::SetBool>("/robot/startup/ready_signal");
    ros::Subscriber sub = nh.subscribe("/robot/startup/ready_signal", 1000, readySignalCallback);

    ros::spin();

    return 0;
}
