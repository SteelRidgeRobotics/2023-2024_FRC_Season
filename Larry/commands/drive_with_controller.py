import typing

import commands2
import constants
import wpilib
from subsystems.swerve_drive import SwerveDrive
from frc6343.controller.deadband import deadband


class DriveWithController(commands2.CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, 
                x: typing.Callable[[], float], y: typing.Callable[[], float],rightx: typing.Callable[[], float],
                leftBumper: typing.Callable[[], bool], rightBumper: typing.Callable[[], bool]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.x = x
        self.y = y
        self.rightx = rightx
        self.leftBumper = leftBumper
        self.rightBumper = rightBumper
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

    def initialize(self) -> None:
        
        self.drive.navX.reset()

    def execute(self) -> None:
        defSpeedMult = self.drive.getDefaultSpeedMultiplier()

        # Modifiers
        if self.leftBumper() and self.rightBumper():
            self.drive.setSpeedMultiplier(defSpeedMult * (constants.kBumperSlowdownFactor / 2))
        elif self.leftBumper() or self.rightBumper():
            self.drive.setSpeedMultiplier(defSpeedMult * constants.kBumperSlowdownFactor)
        else:
            self.drive.setSpeedMultiplier(defSpeedMult)

        wpilib.SmartDashboard.putNumber("Speed Multiplier", self.drive.getSpeedMultiplier())

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        
        return False