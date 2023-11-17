import math
import typing

import commands2
import constants
import wpilib
from subsystems.swerve_drive import SwerveDrive
from frc6343.controller.deadband import deadband


class DriveWithController(commands2.CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, x: typing.Callable[[], float], y: typing.Callable[[], float],
                 rightx: typing.Callable[[], float]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.x = x
        self.y = y
        self.rightx = rightx
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

    def initialize(self) -> None:
        
        self.drive.navX.reset()

    def execute(self) -> None:

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        if translationX == 0 and translationY == 0 and rotationX != 0:
            self.drive.turnInPlace(rotationX)
            return

        self.drive.translateAndTurn(translationX, translationY, rotationX)
        if constants.kDebug:
            wpilib.SmartDashboard.putNumber("translationX", translationX)
            wpilib.SmartDashboard.putNumber("translationY", translationY)
            wpilib.SmartDashboard.putNumber("rotationX", rotationX)

    def end(self, interrupted: bool) -> None:
        
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        
        return False