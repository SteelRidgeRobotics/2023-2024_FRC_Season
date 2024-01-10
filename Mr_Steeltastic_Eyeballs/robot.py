import wpilib
from cscore import CameraServer

class MrSteeltasticsEyes(wpilib.TimedRobot):
    def robotInit(self) -> None:

        wpilib.CameraServer.launch('vision.py:main')
        
        return super().robotInit()
        
    