from commands2 import SequentialCommandGroup
from constants import *
from math import pi
from pathplannerlib import PathPlannerTrajectory
from subsystems.swerve_drive import SwerveDrive
from autos.swerve_drive_controller import SwerveDriveControllerCommand
from wpimath.controller import ProfiledPIDControllerRadians, PIDController

class AutoBase(SequentialCommandGroup):
    profiledThetaController = ProfiledPIDControllerRadians(AutoConstants.kPTheta, 0.0, 0.0, AutoConstants.kThetaControllerRestraints)
    thetaController = PIDController(AutoConstants.kPTheta, 0, 0)

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__()
        self.swerve = swerve
        self.addRequirements(swerve)
        self.thetaController.enableContinuousInput(-pi, pi)

    def baseSwerveCommand(self, trajectory: PathPlannerTrajectory) -> SwerveDriveControllerCommand:
        command = SwerveDriveControllerCommand(trajectory, lambda: self.swerve.getPose(), self.swerve.kinematics,
                                               PIDController(AutoConstants.kPXController, 0, 0),
                                               PIDController(AutoConstants.kPYController, 0, 0), self.profiledThetaController,
                                               lambda moduleStates: self.swerve.setModuleStates(moduleStates), [self.swerve])
        return command
