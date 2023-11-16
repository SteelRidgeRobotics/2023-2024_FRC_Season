from frc6343.controller.guitar.guitar import Guitar
import math
import typing

import commands2
import constants
import conversions
import wpilib
from subsystems.swerve_drive import SwerveDrive

class DriveWithGuitar(commands2.CommandBase):
    """
    Allows driving swerve drive with the guitar controller.

    This is basically DriveByController but the fret buttons set each argument to set values (e.g. greenFret pressed -> rightX = -1)
    """

    def __init__(self, swerveDrive: SwerveDrive, 
                 greenFret: typing.Callable[[], bool], 
                 redFret: typing.Callable[[], bool], 
                 yellowFret: typing.Callable[[], bool], 
                 blueFret: typing.Callable[[], bool],
                 strumBarDown: typing.Callable[[], bool],
                 strumBarUp: typing.Callable[[], bool]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.greenFret = greenFret
        self.redFret = redFret
        self.yellowFret = yellowFret
        self.blueFret = blueFret
        self.strumBarDown = strumBarDown
        self.strumBarUp = strumBarUp
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

    def initialize(self) -> None:
        
        self.drive.navX.reset()

    def execute(self) -> None:

        # Translation X
        if self.redFret() and self.yellowFret():
            translationX = 0
        elif self.redFret():
            translationX = -1
        elif self.yellowFret():
            translationX = 1
        else:
            translationX = 0

        # Translation Y (Strum bar can't return both up and down so the logic is more simple here)
        if self.strumBarUp():
            translationY = -1
        elif self.strumBarDown():
            translationY = 1
        else:
            translationY = 0
        
        # Rotation X
        if self.greenFret() and self.blueFret():
            rotationX = 0
        elif self.greenFret():
            rotationX = -1
        elif self.blueFret():
            rotationX = 1
        else:
            rotationX = 0

        if translationX == 0 and translationY == 0 and rotationX != 0:
            self.drive.turnInPlace(rotationX)
            return

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        
        return False