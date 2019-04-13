#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    def subscribe(message):
        # type: (String) -> None
        text = "You said "+message.data
        pub.publish(text)


    rospy.init_node('demo_repeat', anonymous=False)
    rospy.Subscriber('sound_system/recognition/result', String, subscribe)
    pub = rospy.Publisher('sound_system/speak', String, queue_size=10)

    rospy.spin()
