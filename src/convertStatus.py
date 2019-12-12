#!/usr/bin/env python                                                                               

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from dynamixel_msg.msg import PanTiltStatus
from dynamixel_msg.msg import PanTiltInternalStatus

class Node():
    def __init__(self):
        self.panTickVal = 2048
        self.tiltTickVal = 2048
        self.panTorque = True;
        self.tiltTorque = True;
        self.panMaxSpeed = 0;
        self.tiltMaxSpeed = 0;

    def onPanTiltInternalStatus(self,data):
        self.panTickVal   = (float(data.pan_position)/4095)*360
        self.tiltTickVal  = (float(data.tilt_position - 1024)/4095)*360
        self.panTorque    = data.pan_torque
        self.tiltTorque   = data.tilt_torque
        self.panMaxSpeed  = (float(data.pan_max_speed)/1023)*116.62
        self.tiltMaxSpeed = (float(data.tilt_max_speed)/1023)*116.62


    def convertStatus(self):
        self.panTiltTick = rospy.Publisher('/pan_tilt_status', PanTiltStatus, queue_size=1)

        rospy.Subscriber('pan_tilt_internal_status', PanTiltInternalStatus, self.onPanTiltInternalStatus)
        self.control = PanTiltStatus();
        rate = rospy.Rate(60) # 20hz 
        while not rospy.is_shutdown():
            self.control.header.stamp = rospy.Time.now();
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
        rospy.init_node('convertStatus', anonymous=True)
        node = Node()
        node.convertStatus()
    except rospy.ROSInterruptException:
        pass
