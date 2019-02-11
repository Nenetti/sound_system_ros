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
                    return None
                if sentences[0] == WHYPO_WORD:
                    for sentence in sentences:
                        if WORD in sentence:
                            return sentence.split('"')[1]

    def connect(self):
        try:
            self.client = socket(AF_INET, SOCK_STREAM)
            self.client.connect((self.host, self.port))
            print("Connect Successfully")
            self.pause()
        except IOError:
            sys.exit(1)

    def resume(self):
        self.client.send("RESUME\n")
        self.is_active = True

    def pause(self):
        self.client.send("TERMINATE\n")
        self.is_active = False


if __name__ == '__main__':
    Julius()
