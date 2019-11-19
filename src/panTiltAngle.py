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
        if data.data < 0 :
            data.data = 0
        if data.data > 360 :
            data.data = 360
        self.panTickVal = int((data.data/360)*4095)

    def onTiltAngle(self,data):
        if data.data < 0 :
            data.data = 0
        if data.data > 180 :
            data.data = 180
        self.tiltTickVal = int((data.data/360)*4095 + 1024)


    def panTiltAngle(self):
        #self.panTick = rospy.Publisher('/pan_goal_position', Int16, queue_size=1)
        #self.tiltTick = rospy.Publisher('/tilt_goal_position', Int16, queue_size=1)
        self.panTiltTick = rospy.Publisher('/pan_tilt_control', panTiltControl, queue_size=1)
        
        rospy.Subscriber('pan_goal_angle', Float32, self.onPanAngle)
        rospy.Subscriber('tilt_goal_angle', Float32, self.onTiltAngle)
        self.control = panTiltControl();
        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            # self.panTick.publish(self.panTickVal)
            # self.tiltTick.publish(self.tiltTickVal)
            self.control.pan_position = self.panTickVal;
            self.control.tilt_position = self.tiltTickVal;
            self.panTiltTick.publish(self.control);
            
#            rospy.loginfo("Pan Tick Value: " + str(self.panTickVal))
 #           rospy.loginfo("Tilt Tick Value: " + str(self.tiltTickVal))
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('panTiltAngle', anonymous=True)
        node = Node()
        node.panTiltAngle()
    except rospy.ROSInterruptException:
        pass
