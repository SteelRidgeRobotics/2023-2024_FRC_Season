import typing

from commands.set_driver_profile import MultiplierOptions, TriggerOptions
import commands2
import constants
import wpilib
from subsystems.swerve_drive import SwerveDrive
from frc6343.controller.deadband import deadband

class DriveWithController(commands2.CommandBase):
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

        self.defSpeedMult = self.drive.getDefaultSpeedMultiplier()
        self.defRotMult = self.drive.getDefaultRotationMultiplier()
        self.defTranslationMult = self.drive.getDefaultTranslationMultiplier()

        self.leftBumperMode = self.drive.getLeftBumperMode()
        self.rightBumperMode = self.drive.getRightBumperMode()

        self.leftBumperFactor = self.drive.getLeftBumperFactor()
        self.rightBumperFactor = self.drive.getRightBumperFactor()

        self.leftTriggerMode = self.drive.getLeftTriggerMode()
        self.leftTriggerOption = self.drive.getLeftTriggerOption()
        self.rightTriggerMode = self.drive.getRightTriggerMode()
        self.rightTriggerOption = self.drive.getRightTriggerOption()

    def execute(self) -> None:
        
        # Bumpers
        if self.leftBumper():
            setMultiplierFromOption(self.drive, self.leftBumperMode, self.leftBumperFactor)
        elif not self.rightBumper():
            setMultiplierFromOption(self.drive, self.leftBumperMode, 1)

        if self.rightBumper():
            setMultiplierFromOption(self.drive, self.rightBumperMode, self.rightBumperFactor)
        elif not self.leftBumper():
            setMultiplierFromOption(self.drive, self.rightBumperMode, 1)

        combinedFactor = self.drive.getRightBumperFactor() * self.drive.getLeftBumperFactor()
        if self.leftBumperMode == self.rightBumperMode and self.leftBumper() and self.rightBumper():
            setMultiplierFromOption(self.drive, self.leftBumperMode, combinedFactor)

        # Triggers
        setMultiplierFromTriggerOption(self.drive, self.leftBumperMode, self.leftTriggerOption, self.leftTrigger())
        setMultiplierFromTriggerOption(self.drive, self.rightBumperMode, self.rightTriggerOption, self.rightTrigger())

        wpilib.SmartDashboard.putNumber("Speed Mult.", self.drive.getSpeedMultiplier())
        wpilib.SmartDashboard.putNumber("Rot Mult.", self.drive.getRotationMultiplier())
        wpilib.SmartDashboard.putNumber("Trans Mult.", self.drive.getTranslationMultiplier())

        translationX = deadband(self.x(), constants.kdeadband)
        translationY = deadband(self.y(), constants.kdeadband)
        rotationX = deadband(self.rightx(), constants.kdeadband)

        self.drive.translateAndTurn(translationX, translationY, rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        
        return False
    
def setMultiplierFromOption(swerve: SwerveDrive, option: MultiplierOptions, factor: float) -> None:
    match option:
        case MultiplierOptions.NONE:
            pass
        case MultiplierOptions.ROTATION:
            swerve.setRotationMultiplier(swerve.getDefaultRotationMultiplier() * factor)
        case MultiplierOptions.TRANSLATION:
            swerve.setTranslationMultiplier(swerve.getDefaultTranslationMultiplier() * factor)
        case MultiplierOptions.SPEED:
            swerve.setSpeedMultiplier(swerve.getDefaultSpeedMultiplier() * factor)

def setMultiplierFromTriggerOption(swerve: SwerveDrive, multOption: MultiplierOptions, option: TriggerOptions, trigger: float) -> None:
    match option:
        case TriggerOptions.NONE:
            pass
        case TriggerOptions.ANALOG_1_2:
            setMultiplierFromOption(swerve, multOption, trigger + 1)
        case TriggerOptions.ANALOG_0_1:
            setMultiplierFromOption(swerve, multOption, trigger)
        case TriggerOptions.ANALOG_0_05:
            setMultiplierFromOption(swerve, multOption, trigger / 2)
        case TriggerOptions.ANALOG_0_025:
            setMultiplierFromOption(swerve, multOption, trigger / 4)
        case TriggerOptions.SET_2:
            setMultiplierFromOption(swerve, multOption, 2 if trigger > 0.9 else 1)
        case TriggerOptions.SET_05:
            setMultiplierFromOption(swerve, multOption, 0.5 if trigger > 0.9 else 1)
        case TriggerOptions.SET_025:
            setMultiplierFromOption(swerve, multOption, 0.25 if trigger > 0.9 else 1)
