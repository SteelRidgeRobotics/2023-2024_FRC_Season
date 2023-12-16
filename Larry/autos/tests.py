from autos.base import AutoBase
from commands2 import InstantCommand
from constants import *
from pathplannerlib import PathPlanner
from subsystems.swerve_drive import SwerveDrive

class TestForward(AutoBase):
    """
    Moves the robot 1 meter forward using a PathPlanner trajectory.
    """

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__(swerve)

        path = PathPlanner.loadPath("TestForward", kdefaultPathConstraints)
        command1 = self.baseSwerveCommand(path)
        initialState = command1.pose()

        self.addCommands(
            InstantCommand(lambda: swerve.navX.reset()),
            # Reset odometry here (TODO)
            command1
        )
