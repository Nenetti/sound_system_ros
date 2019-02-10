#!/usr/bin/env python
# -*- coding: utf-8 -*-

from module.julius import Julius
from module.se import SE
import rospy
from std_msgs.msg import String
import subprocess


def start():
    def resume(data):
        # type: (String) -> None
        module.resume()
        print(SE.START)
        SE.play(SE.START)


    def pause(data):
        # type: (String) -> None
        module.pause()
        SE.play(SE.STOP)

    rospy.init_node('voice_recognition', anonymous=False)
    rospy.Subscriber("sound_system/resume", String, resume)
    rospy.Subscriber("sound_system/pause", String, pause)
    nlp_pub = rospy.Publisher('natural_language/text', String, queue_size=10)

    module = Julius()

    while True:
        result = module.recognition()
        if result is not None:
            nlp_pub.publish(String(result))






if __name__ == '__main__':
    start()
