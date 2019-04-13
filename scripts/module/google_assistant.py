#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, String
from sound_system.srv import *

sys.path.append('/home/ubuntu/catkin_ws/src/sound_system_ros/scripts/module/google')
import main_assistant


# 引数は en-US か ja_jp
def start(lang="en-US", debug=True, answer=False):
    def activate(message):
        # type: (Bool) -> None
        if message.data:
            actitive = True
        else:
            actitive = False

    def response(message):
        # type: (StatusRequest) -> StatusResponse
        return actitive

    def recognition(message):
        # type: (RecognitionRequest) -> RecognitionResponse
        actitive = True
        rospy.loginfo("Recognition Start")
        result, answer = assistant.start()
        rospy.loginfo("Result -> %s" % result)
        rospy.loginfo("Answer -> %s" % answer)
        actitive = False
        return RecognitionResponse(result)

    rospy.init_node('google_assistant', anonymous=False)
    rospy.Subscriber('sound_system/module/recognition/activate', Bool, activate)
    rospy.Service("sound_system/module/recognition/status", Status, response)
    rospy.Service("sound_system/module/recognition/request", Recognition, recognition)
    r = rospy.Rate(10)

    assistant = main_assistant.main(lang, debug, answer)
    actitive = False

    rospy.spin()


if __name__ == '__main__':
    start()
