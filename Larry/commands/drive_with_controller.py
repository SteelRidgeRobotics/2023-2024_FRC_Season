import math
import typing

import commands2
import constants
import conversions
import wpilib
from subsystems.swerve_drive import SwerveDrive


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

        self.gyroAngle = self.drive.getYaw() + 180
        wpilib.SmartDashboard.putNumber("Yaw", self.gyroAngle)
        self.angle = (360 - (conversions.convertJoystickInputToDegrees(conversions.deadband(self.x(), constants.kdeadband),
                                                               conversions.deadband(self.y(), constants.kdeadband)) + self.gyroAngle)) % 360
        wpilib.SmartDashboard.putNumber("Adjusted Angle", self.angle)
        self.magnitude = math.hypot(conversions.deadband(self.x(), constants.kdeadband),
                                    conversions.deadband(self.y(), constants.kdeadband))

        # self.magnitude *= 0.5
        # self.angle -= self.drive.getYaw()

        if self.magnitude >= 1.0:
            self.magnitude = 1.0

        if self.magnitude == 0.0:
            # only rotation
            self.drive.turnInPlace(conversions.deadband(self.rightx(), constants.kdeadband))
        else:
            self.drive.translate(self.angle, self.magnitude)
            """
            if self.magnitude != 0.0 and self.rightx != 0.0:
                # checks if both joysticks are being used
                self.drive.moveWhileSpinning(self.x(), self.y(), self.rightx())
            else:
                # no rotation wanted
                self.drive.translate(self.angle, self.magnitude)
                """

        

    def end(self, interrupted: bool) -> None:
        
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        
        return False