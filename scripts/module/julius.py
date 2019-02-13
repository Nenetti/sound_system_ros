#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospkg
import rospy
from time import sleep
from atexit import register
from threading import Thread
import subprocess
from socket import socket, AF_INET, SOCK_STREAM
from re import compile
from atexit import register
from std_msgs.msg import Bool, String
from sound_system.srv import *
import signal

RECOGOUT_START = "<RECOGOUT>"
RECOGOUT_END = "</RECOGOUT>"
REJECT = "<REJECTED"
WHYPO_WORD = "<WHYPO"
WORD = "WORD="
PACKAGE = rospkg.RosPack().get_path('sound_system')


class Julius:
    def __init__(self, host, port, config, is_debug=False):
        # type: (str, int) -> None
        self.is_debug = is_debug
        self.host = host
        self.port = port
        self.client = None
        self.process = None
        self.is_active = True

        self.boot(config)
        sleep(1)
        self.connect()

        def exit(signal, frame):
            print("\n")
            print("process exit [PID=%d]" % self.process.pid)
            self.process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, exit)

    def boot(self, config="session_en.jconf"):
        if self.is_debug:
            self.process = subprocess.Popen([PACKAGE + "/julius/boot.sh", config, str(self.port)])
        else:
            self.process = subprocess.Popen([PACKAGE + "/julius/boot.sh", config, str(self.port)],
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def resume(self):
        self.client.send("RESUME\n")
        self.is_active = True

    def pause(self):
        self.client.send("TERMINATE\n")
        self.is_active = False

    def recognition(self):
        # type: ()-> str
        data = " "
        is_recogout = False
        while True:
            recv = self.client.recv(2048).decode("utf-8")
            if self.is_debug:
                print(recv)
            if not len(recv) > 0:
                continue
            if data[-1] is "\n":
                data = recv
            else:
                data += recv
            if not data[-1] == "\n":
                continue
            lines = data.split("\n")
            for line in lines:
                if not len(line) > 1:
                    continue
                sentences = line.lstrip().split()
                if not len(sentences) > 0:
                    continue
                if sentences[0] == RECOGOUT_START:
                    is_recogout = True
                if (sentences[0] == REJECT) or (is_recogout and sentences[0] == RECOGOUT_END):
                    continue
                    #return None
                if sentences[0] == WHYPO_WORD:
                    for sentence in sentences:
                        if WORD in sentence:
                            return sentence.split('"')[1].replace("_", " ")

    def connect(self):
        try:
            self.client = socket(AF_INET, SOCK_STREAM)
            self.client.connect((self.host, self.port))
            print("Connect Successfully")
            self.pause()
        except IOError:
            sys.exit(1)


if __name__ == '__main__':

    def activate(message):
        # type: (Bool) -> None
        if message.data:
            j.resume()
        else:
            j.pause()


    def response(message):
        # type: (StatusRequest) -> StatusResponse
        return StatusResponse(j.is_active)


    def recognition(message):
        # type: (RecognitionRequest) -> RecognitionResponse
        j.resume()
        result = RecognitionResponse(j.recognition())
        j.pause()
        return result


    rospy.init_node('julius', anonymous=False)
    rospy.Subscriber('sound_system/module/recognition/activate', Bool, activate)
    rospy.Service("sound_system/module/recognition/status", Status, response)
    rospy.Service("sound_system/module/recognition/request", Recognition, recognition)
    # pub = rospy.Publisher("sound_system/module/recognition/result", String, queue_size=10)

    config = rospy.get_param("/julius/config")
    host = rospy.get_param("/julius/host")
    port = rospy.get_param("/julius/port")
    is_debug = rospy.get_param("/julius/debug")

    j = Julius(host, port, config, is_debug)

    rospy.spin()
