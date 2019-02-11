#!/usr/bin/env python
# -*- coding: utf-8 -*-

from module.julius import Julius
from module.se import SE
from module.svox import SVOX
import rospy
import rosparam
from std_msgs.msg import String
import subprocess
import os
import rospkg

def start():
    def resume(message=None):
        # type: (String) -> None
        if not module.is_active:
            module.resume()
            SE.play(SE.START)

    def pause(message=None):
        # type: (String) -> None
        if module.is_active:
            module.pause()
            SE.play(SE.STOP)

    def speak(message):
        # type: (String) -> None
        pause()
        SVOX.play(message.data)
        resume()

    rospy.init_node('sound_system', anonymous=False)
    rospy.set_param("/pkg", rospkg.RosPack().get_path('sound_system'))

    rospy.Subscriber("sound_system/resume", String, resume)
    rospy.Subscriber("sound_system/pause", String, pause)
    rospy.Subscriber("sound_system/speak", String, speak)
    reco_pub = rospy.Publisher('sound_system/recognition', String, queue_size=10)

    config = rospy.get_param("/sound_system/config")
    host = rospy.get_param("/sound_system/host")
    port = rospy.get_param("/sound_system/port")
    is_debug = rospy.get_param("/sound_system/debug")

    module = Julius(host, port, config, is_debug)
    
    while True:
        if module.is_active:
            result = module.recognition()
            if result is not None:
                result = result.replace("_", " ")
                reco_pub.publish(String(result))
                if is_debug:
                    print("RESULT: %s" % result)


if __name__ == '__main__':
    start()
