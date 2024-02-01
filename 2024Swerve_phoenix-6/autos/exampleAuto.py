from commands2 import InstantCommand, SequentialCommandGroup, SwerveControllerCommand
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from Constants import *
from subsystems.Swerve import *


class ExampleAuto(SequentialCommandGroup):
    def exampleAuto(self, s_Swerve:Swerve):
        config = TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Constants.Swerve.swerveKinematics)

        # An example trajectory to follow.  All units in meters.
        self.example_trajectory = Trajectory
        self.example_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(1, 1), Translation2d(2, -1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(3, 0, Rotation2d(0)),
            config
        )

        theta_controller = ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)  # Assuming Math.PI in Java

        swerve_controller_command = SwerveControllerCommand(
            self.example_trajectory,
            s_Swerve.getPose,
            Constants.Swerve.swerveKinematics,
            PIDController(Constants.AutoConstants.kPXController, 0, 0),
            PIDController(Constants.AutoConstants.kPYController, 0, 0),
            theta_controller,
            s_Swerve.setModuleStates,
            s_Swerve
        )

        
        self.addCommands(

            InstantCommand(lambda: s_Swerve.setPose(self.example_trajectory.initialPose())),
            swerve_controller_command
        )
