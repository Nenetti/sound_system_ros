import rospkg
import subprocess
import rospy

PATH = rospy.get_param("/pkg") + "/SE"

class SE():
    START = PATH + "/" + "start.wav"
    STOP = PATH + "/" + "stop.wav"

    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)