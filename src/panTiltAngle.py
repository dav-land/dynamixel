#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamixel.msg import panTiltControl

class Node():
    def __init__(self):
        self.panTickVal = 2048
        self.tiltTickVal = 2048

    def onPanAngle(self,data):
        self.panTickVal = int((data.data/360)*4095)

    def onTiltAngle(self,data):
        self.tiltTickVal = int((data.data/360)*4095 + 1024)


    def panTiltAngle(self):
        self.panTiltTick = rospy.Publisher('/pan_tilt_control', panTiltControl, queue_size=1)
        
        rospy.Subscriber('pan_goal_angle', Float32, self.onPanAngle)
        rospy.Subscriber('tilt_goal_angle', Float32, self.onTiltAngle)
        self.control = panTiltControl();
        self.control.pan_max_speed = 0;
        self.control.pan_torque = True;
        self.control.tilt_max_speed = 0;
        self.control.tilt_torque = True;
        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            self.control.pan_position = self.panTickVal;
            self.control.tilt_position = self.tiltTickVal;
            self.panTiltTick.publish(self.control);
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('panTiltAngle', anonymous=True)
        node = Node()
        node.panTiltAngle()
    except rospy.ROSInterruptException:
        pass
