from commands2 import CommandBase
from constants import *
from frc6343.controller.deadband import deadband
from subsystems.swerve_drive import SwerveDrive
from typing import Callable
from wpilib import SmartDashboard

class DriveControllerDefault(CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, 
                x: Callable[[], float], y: Callable[[], float],rightx: Callable[[], float],
                leftBumper: Callable[[], bool], rightBumper: Callable[[], bool],
                decreaseAngleOffset: Callable[[], bool], increaseAngleOffset: Callable[[], bool]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.x = x
        self.y = y
        self.rightx = rightx
        self.leftBumper = leftBumper
        self.rightBumper = rightBumper
        self.decreaseAngleOffset = decreaseAngleOffset
        self.increaseAngleOffset = increaseAngleOffset
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

        self.drive.navX.reset()

    def initialize(self) -> None:
        self.drive.setRotationMultiplier(1)
        self.drive.setTranslationMultiplier(1)

        self.defSpeedMult = 1

        self.leftBumperFactor = self.rightBumperFactor = 0.5

        self.angleDecreasePressed = self.angleIncreasePressed = False

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

        SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        # Angle Offset
        if self.decreaseAngleOffset() and not self.angleDecreasePressed:
            self.drive.addToAngleOffset(-15)
            self.angleDecreasePressed = True
        elif not self.decreaseAngleOffset():
            self.angleDecreasePressed = False

        if self.increaseAngleOffset() and not self.angleIncreasePressed:
            self.drive.addToAngleOffset(15)
            self.angleIncreasePressed = True
        elif not self.increaseAngleOffset():
            self.angleIncreasePressed = False

        translationX = deadband(self.x(), kdeadband)
        translationY = deadband(self.y(), kdeadband)
        rotationX = deadband(self.rightx(), kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return True if self.drive.getDefaultCommand() != self else False
    
class DriveControllerDefaultSlow(CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, 
                x: Callable[[], float], y: Callable[[], float],rightx: Callable[[], float],
                leftBumper: Callable[[], bool], rightBumper: Callable[[], bool],
                decreaseAngleOffset: Callable[[], bool], increaseAngleOffset: Callable[[], bool]) -> None:
        
        super().__init__()
        self.drive = swerveDrive
        self.x = x
        self.y = y
        self.rightx = rightx
        self.leftBumper = leftBumper
        self.rightBumper = rightBumper
        self.decreaseAngleOffset = decreaseAngleOffset
        self.increaseAngleOffset = increaseAngleOffset
        self.addRequirements([self.drive])
        self.drive.reset()
        self.drive.getPosFromOffState()

    def initialize(self) -> None:
        self.drive.setRotationMultiplier(1)
        self.drive.setTranslationMultiplier(1)

        self.defSpeedMult = 0.25

        self.leftBumperFactor = self.rightBumperFactor = 2

        self.angleDecreasePressed = self.angleIncreasePressed = False

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

        SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        # Angle Offset
        if self.decreaseAngleOffset() and not self.angleDecreasePressed:
            self.drive.addToAngleOffset(-15)
            self.angleDecreasePressed = True
        elif not self.decreaseAngleOffset():
            self.angleDecreasePressed = False

        if self.increaseAngleOffset() and not self.angleIncreasePressed:
            self.drive.addToAngleOffset(15)
            self.angleIncreasePressed = True
        elif not self.increaseAngleOffset():
            self.angleIncreasePressed = False

        translationX = deadband(self.x(), kdeadband)
        translationY = deadband(self.y(), kdeadband)
        rotationX = deadband(self.rightx(), kdeadband)

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
class DriveControllerWyatt(CommandBase):
    def __init__(self, swerveDrive: SwerveDrive, 
                x: Callable[[], float], y: Callable[[], float],rightx: Callable[[], float],
                leftBumper: Callable[[], bool], rightBumper: Callable[[], bool],
                leftTrigger: Callable[[], float], rightTrigger: Callable[[], float]) -> None:
        
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

        SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        SmartDashboard.putNumber("Translation Mult.", self.drive.getTranslationMultiplier())

        translationX = deadband(self.x(), kdeadband)
        translationY = deadband(self.y(), kdeadband)
        rotationX = deadband(self.rightx(), kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return True if self.drive.getDefaultCommand() != self else False
