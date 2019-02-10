import rospkg
import subprocess

PATH = rospkg.RosPack().get_path('sound_system_ros') + "/SE"

class SE():
    START = PATH + "/" + "start.wav"
    STOP = PATH + "/" + "stop.wav"

    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE)