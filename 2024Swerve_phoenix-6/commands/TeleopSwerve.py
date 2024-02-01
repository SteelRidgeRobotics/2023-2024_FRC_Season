from commands2 import Command
from wpimath import *
from wpimath.geometry import Translation2d
from Constants import *
from subsystems.Swerve import *
from robot import *

class TeleopSwerve(Command):
    def __init__(self, s_Swerve:Swerve, translationSup:float, strafeSup:float, rotationSup:float, robotCentricSup:bool):
        super().__init__()
        self.s_Swerve = s_Swerve
        self.requirements(s_Swerve)

        self.translationSup = translationSup
        self.strafeSup = strafeSup
        self.rotationSup = rotationSup
        self.robotCentricSup = robotCentricSup

    def execute(self):
        # Get Values, Deadband
        translationVal = applyDeadband(self.translationSup(), Constants.stickDeadband)  # Replace 0.1 with the actual deadband value
        strafeVal = applyDeadband(self.strafeSup(), Constants.stickDeadband)  # Replace 0.1 with the actual deadband value
        rotationVal = applyDeadband(self.rotationSup(), Constants.stickDeadband)  # Replace 0.1 with the actual deadband value

        # Drive
        translation = Translation2d(translationVal, strafeVal) * Constants.Swerve.maxSpeed
        angular_velocity = rotationVal * 1.0  # Replace 1.0 with the actual max angular velocity value

        self.s_Swerve.drive(translation, angular_velocity, not self.robotCentricSup(), True)
