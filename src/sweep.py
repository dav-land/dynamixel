#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamixel_msg.msg import PanTiltControl

class Node():
    def __init__(self):
        self.panTickVal = 20
        self.tiltTickVal = 140

   
    def panTiltAngle(self):
        self.panTilt = rospy.Publisher('pan_tilt_control', PanTiltControl, queue_size=1)
        rate = rospy.Rate(5) # 10hz
        negative_pan = 1
        negative_tilt = 1
        while not rospy.is_shutdown():
            if self.panTickVal >= 360 or self.panTickVal <= 0:
                negative_pan = -1*negative_pan
                
            self.panTickVal = self.panTickVal + 20*negative_pan
           
                        
            if self.panTickVal >= 360 or self.panTickVal <= 0:
                if self.tiltTickVal >= 180 or self.tiltTickVal <= 0:
                    negative_tilt = -1*negative_tilt

                self.tiltTickVal = self.tiltTickVal + 20*negative_tilt

            self.control = PanTiltControl();
            self.control.pan_position = self.panTickVal;
            self.control.tilt_position = self.tiltTickVal;
            self.control.pan_torque = True;
            self.control.tilt_torque = True;
            self.control.pan_max_speed = 0;
            self.control.tilt_max_speed = 0;
            self.panTilt.publish(self.control)
            rospy.loginfo("Pan Tick Value: " + str(self.panTickVal))
            rospy.loginfo("Tilt Tick Value: " + str(self.tiltTickVal))
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('panTiltSweep', anonymous=True)
        node = Node()
        node.panTiltAngle()
    except rospy.ROSInterruptException:
        pass
