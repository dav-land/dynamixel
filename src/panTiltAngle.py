#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamixel_msg.msg import PanTiltControl
from dynamixel_msg.msg import PanTiltInternalControl

class Node():
    def __init__(self):
        self.panTickVal = 2048
        self.tiltTickVal = 2048
        self.panTorque = True;
        self.tiltTorque = True;
        self.panMaxSpeed = 0;
        self.tiltMaxSpeed = 0;

    def onPanTiltControl(self,data):
        self.panTickVal   = int((data.pan_position/360)*4095)
        self.tiltTickVal  = int((data.tilt_position/360)*4095 + 1024)
        self.panTorque    = data.pan_torque
        self.tiltTorque   = data.tilt_torque
        self.panMaxSpeed  = int((data.pan_max_speed/116.62)*1023)
        self.tiltMaxSpeed = int((data.tilt_max_speed/116.62)*1023)


    def panTiltAngle(self):
        self.panTiltTick = rospy.Publisher('/pan_tilt_internal_control', PanTiltInternalControl, queue_size=1)
        
        rospy.Subscriber('pan_tilt_control', PanTiltControl, self.onPanTiltControl)
        self.control = PanTiltInternalControl();
        rate = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():
            self.control.pan_position = self.panTickVal;
            self.control.tilt_position = self.tiltTickVal;
            self.control.pan_max_speed = self.panMaxSpeed;
            self.control.tilt_max_speed = self.tiltMaxSpeed;
            self.control.pan_torque = self.panTorque;
            self.control.tilt_torque = self.tiltTorque;
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
