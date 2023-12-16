from commands2 import CommandBase, Subsystem
from typing import Callable
from pathplannerlib import PathPlannerTrajectory
from wpilib import SmartDashboard, Timer
from wpimath.controller import HolonomicDriveController, ProfiledPIDControllerRadians, PIDController
from wpimath.geometry import *
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState

class SwerveDriveControllerCommand(CommandBase):
    """
    Takes 2 PID controllers and 1 ProfiledPIDController to follow a trajectory using swerve drive.
    """

    def __init__(self, trajectory: PathPlannerTrajectory, pose: Callable[[], Pose2d], kinematics: SwerveDrive4Kinematics,
                 xController: PIDController, yController: PIDController, thetaController: ProfiledPIDControllerRadians,
                 outputModuleStates: Callable[[tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]], None], requirements: tuple[Subsystem]) -> None:
        super().__init__()

        self.timer = Timer()
        self.trajectory = trajectory
        self.pose = pose
        self.kinematics = kinematics
        self.controller = HolonomicDriveController(xController, yController, thetaController)
        self.outputModuleStates = outputModuleStates

        #self.addRequirements(*requirements)

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()
    
    def execute(self) -> None:
        currentTime = self.timer.get()

        desiredState = self.trajectory.sample(currentTime)

        targetChassisSpeeds = self.controller.calculate(self.pose(), desiredState.asWPILibState(), desiredState.holonomicRotation)
        targetModuleStates = self.kinematics.toSwerveModuleStates(targetChassisSpeeds)

        SmartDashboard.putNumber("Velocity", desiredState.velocity)
        SmartDashboard.putNumber("Angular Velocity", desiredState.holonomicAngularVelocity)
        SmartDashboard.putNumber("Acceleration", desiredState.acceleration)

        self.outputModuleStates(targetModuleStates)
    
    def end(self, interrupted: bool) -> None:
        self.timer.stop()
    
    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.getTotalTime())
    