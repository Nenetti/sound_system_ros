import subprocess
import rospkg
import rospy

PATH = rospy.get_param("/pkg") + "/voice"

class SVOX:

    @staticmethod
    def play(text, output="voice.wav"):
        # type: (str, str) -> None
        file = PATH + "/" + output
        subprocess.call(["pico2wave", "-w=" + file, text], stdout=subprocess.PIPE)
        subprocess.call(["aplay", file], stdout=subprocess.PIPE)
