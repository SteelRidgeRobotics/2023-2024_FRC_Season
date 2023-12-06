import typing

import commands2
import constants
import wpilib
from subsystems.swerve_drive import SwerveDrive
from frc6343.controller.deadband import deadband

class DriveControllerDefault(commands2.CommandBase):
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

        self.drive.navX.reset()

    def initialize(self) -> None:
        self.drive.setRotationMultiplier(1)
        self.drive.setTranslationMultiplier(1)

        self.defSpeedMult = 1

        self.leftBumperFactor = self.rightBumperFactor = 0.5

    def execute(self) -> None:
        
        # Bumpers
        if self.leftBumper() and self.rightBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.leftBumperFactor * self.rightBumperFactor)
        elif self.leftBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.leftBumperFactor)
        elif self.rightBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.rightBumperFactor)
        else:
            self.drive.setSpeedMultiplier(self.defSpeedMult)

        wpilib.SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        wpilib.SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        wpilib.SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return True if self.drive.getDefaultCommand() != self else False
    
class DriveControllerDefaultSlow(commands2.CommandBase):
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

        self.drive.setRotationMultiplier(1)
        self.drive.setTranslationMultiplier(1)

        self.defSpeedMult = 0.25

        self.leftBumperFactor = self.rightBumperFactor = 2

    def execute(self) -> None:
        
        # Bumpers
        if self.leftBumper() and self.rightBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.leftBumperFactor * self.rightBumperFactor)
        elif self.leftBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.leftBumperFactor)
        elif self.rightBumper():
            self.drive.setSpeedMultiplier(self.defSpeedMult * self.rightBumperFactor)
        else:
            self.drive.setSpeedMultiplier(self.defSpeedMult)

        wpilib.SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        wpilib.SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        wpilib.SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return True if self.drive.getDefaultCommand() != self else False
    
"""
idk what the variable default speed does but translational default 25% and rotation default 100%
bumpers sets rotation speed to 25%
triggers speed up translational based on how much they're pressed

summary I guess ^^^^^ 
"""
class DriveControllerWyatt(commands2.CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, 
                x: typing.Callable[[], float], y: typing.Callable[[], float],rightx: typing.Callable[[], float],
                leftBumper: typing.Callable[[], bool], rightBumper: typing.Callable[[], bool],
                leftTrigger: typing.Callable[[], float], rightTrigger: typing.Callable[[], float]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.x = x
        self.y = y
        self.rightx = rightx
        self.leftBumper = leftBumper
        self.rightBumper = rightBumper
        self.leftTrigger = leftTrigger
        self.rightTrigger = rightTrigger
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

    def initialize(self) -> None:
        
        self.drive.navX.reset()

        self.drive.setSpeedMultiplier(1)

        self.defRotMult = 1
        self.defTranslationMult = 0.25

    def execute(self) -> None:
        
        # Bumpers
        if self.leftBumper() or self.rightBumper():
            self.drive.setRotationMultiplier(self.defRotMult / 4)
        else:
            self.drive.setRotationMultiplier(self.defRotMult)

        # Triggers
        maxTrigger = max(self.leftTrigger(), self.rightTrigger())
        self.drive.setTranslationMultiplier(maxTrigger * 0.75 + self.defTranslationMult)

        wpilib.SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        wpilib.SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        wpilib.SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return True if self.drive.getDefaultCommand() != self else False
