#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool, SetBoolResponse 

def ready_signal_callback(request):
    rospy.loginfo("Ready signal successfully received.")
    return SetBoolResponse(success=True, message="Ready signal set to True.") 

def main():
    rospy.init_node('main_ready')
    
    # Corrected the service type and added missing parenthesis
    service = rospy.Service('/robot/startup/ready_signal', SetBool, ready_signal_callback)
    pub = rospy.Publisher('/robot/startup/ready_signal', PointStamped, queue_size=10)  #
    
    rospy.loginfo("Ready signal successfully sent out.")
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        ready_signal = PointStamped()
        ready_signal.header.stamp = rospy.Time.now()
        ready_signal.header.frame_id = "inspection"
        pub.publish(ready_signal)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
