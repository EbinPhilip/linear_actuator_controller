#!/usr/bin/env python

import rospy
from linear_actuator_controller.msg import LinearActuatorInput
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('input', LinearActuatorInput, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = LinearActuatorInput()
    msg.position = 12000
    msg.speed = 2400.0
    msg.acceleration_max = 4800.0
    msg.position_max = 48000
    msg.speed_max = 4800.0
    msg.enabled = True
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass