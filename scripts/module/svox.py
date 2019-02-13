#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import rospkg
import rospy
from sound_system.srv import *
from std_msgs.msg import Bool

PATH = rospkg.RosPack().get_path('sound_system') + "/voice"
FILE = "voice.wav"


if __name__ == '__main__':

    def speak(message):
        # type: (SpeakRequest) -> SpeakResponse
        file = PATH + "/" + FILE
        subprocess.call(["pico2wave", "-w=" + file, message.text], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        subprocess.call(["aplay", file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return SpeakResponse(True)

    rospy.init_node('svox', anonymous=False)
    rospy.Service("sound_system/module/speak", Speak, speak)

    rospy.spin()
