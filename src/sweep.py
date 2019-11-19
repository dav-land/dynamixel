#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class Node():
    def __init__(self):
        self.panTickVal = 20
        self.tiltTickVal = 140

   
    def panTiltAngle(self):
        self.panTick = rospy.Publisher('pan_goal_angle', Float32, queue_size=1)
        self.tiltTick = rospy.Publisher('tilt_goal_angle', Float32, queue_size=1)
        rate = rospy.Rate(5) # 10hz
        negative_pan = 1
        negative_tilt = 1
        while not rospy.is_shutdown():
            if self.panTickVal == 360 or self.panTickVal == 0:
                negative_pan = -1*negative_pan
                
            self.panTickVal = self.panTickVal + 20*negative_pan
           
                        
            if self.panTickVal == 360 or self.panTickVal == 0:
                if self.tiltTickVal == 180 or self.tiltTickVal == 0:
                    negative_tilt = -1*negative_tilt

                self.tiltTickVal = self.tiltTickVal + 20*negative_tilt
            
            self.panTick.publish(self.panTickVal)
            self.tiltTick.publish(self.tiltTickVal)
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
