/**
 *
 * @file safety-ab.cpp
 * @brief
 * Safety filter node for evaluating the safety status of two different systems (diff_A and diff_B) 
 * and publishing the combined safety status. The node subscribes to safety status topics of both systems, 
 * evaluates the safety status based on logical AND operation, and publishes the combined safety status.
 * 
 * Functionalities :
 * - Subscribing to safety status topics of diff_A and diff_B
 * - Evaluating safety status based on logical AND operation
 * - Publishing the combined safety status
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
#include "std_msgs/Bool.h"
#include <vector>

class SafetyFilter {
public:
    SafetyFilter() : nh_("~"), diff_A_state_(false), diff_B_state_(false) {
        // Initialize subscribers
        sub_diff_A_ = nh_.subscribe("/diff_A/ladybug/safe", 10, &SafetyFilter::diffACallback, this);
        sub_diff_B_ = nh_.subscribe("/diff_B/ladybug/safe", 10, &SafetyFilter::diffBCallback, this);

        // Initialize publisher
        pub_ = nh_.advertise<std_msgs::Bool>("/robot/object/ladybug/safe", 10);
    }

    void diffACallback(const std_msgs::Bool::ConstPtr& msg) {
        diff_A_state_ = msg->data;
    }

    void diffBCallback(const std_msgs::Bool::ConstPtr& msg) {
        diff_B_state_ = msg->data;
    }

    void evaluateAndPublish() {
        std_msgs::Bool output;
        output.data = diff_A_state_ && diff_B_state_;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_diff_A_;
    ros::Subscriber sub_diff_B_;
    ros::Publisher pub_;

    bool diff_A_state_ = true;
    bool diff_B_state_ = true;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ladybug_safety_filter");
    SafetyFilter filter;

    ros::Rate rate(6); 
    while (ros::ok()) {
        ros::spinOnce();
        filter.evaluateAndPublish(); 
        rate.sleep(); 
    }

    return 0;
}