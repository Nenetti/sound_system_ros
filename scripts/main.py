#!/usr/bin/env python
# -*- coding: utf-8 -*-

from module.se import SE
import rospy
import rosparam
from std_msgs.msg import String, Bool
import subprocess
import os
import rospkg
from sound_system.srv import *


def start():
    def activate(message):
        status = get_status()
        if status and not message.data:
            pause()
            active_pub.publish(message)
        elif not status and message.data:
            resume()
            active_pub.publish(message)

    def get_status():
        rospy.wait_for_service("sound_system/module/recognition/status")
        request = rospy.ServiceProxy("sound_system/module/recognition/status", Status)
        response = request()
        return response.active

    def resume():
        SE.play(SE.START)

    def pause():
        SE.play(SE.STOP)

    def speak(message):
        # type: (String) -> None
        #if get_status():
        #    pause()
        rospy.wait_for_service("sound_system/module/speak")
        request = rospy.ServiceProxy("sound_system/module/speak", Speak)
        request(message.data)

    def recognition(message):
        activate(Bool(True))
        rospy.wait_for_service("sound_system/module/recognition/request")
        request = rospy.ServiceProxy("sound_system/module/recognition/request", Recognition)
        response = request()
        activate(Bool(False))
        speak(String(response.result))
        return RecognitionResponse(response.result)

    rospy.init_node('sound_system', anonymous=False)

    rospy.Subscriber("sound_system/recognition/activate", Bool, activate)
    rospy.Subscriber("sound_system/speak", String, speak)

    active_pub = rospy.Publisher("sound_system/module/recognition/activate", Bool, queue_size=10)
    rospy.Service("sound_system/recognition", Recognition, recognition)
    rospy.Subscriber("sound_system/module/recognition/request2", String, recognition)

    rospy.spin()


if __name__ == '__main__':
    start()
