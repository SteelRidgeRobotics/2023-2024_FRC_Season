import wpilib
from cscore import CameraServer
import cv2

class MrSteeltasticsEyes(wpilib.TimedRobot):
    def robotInit(self) -> None:
        
        wpilib.CameraServer.launch('vision2.py:main')
        
        return super().robotInit()

    def __init__(self):
        pass
if __name__ == "__main__":
    wpilib.run(MrSteeltasticsEyes)
    