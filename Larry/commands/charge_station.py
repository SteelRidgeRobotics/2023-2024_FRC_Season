from commands2 import CommandBase
from constants import *
from subsystems.swerve_drive import SwerveDrive
from wpilib import SmartDashboard
from wpimath.controller import PIDController

class ChargeStation(CommandBase):

    def __init__(self, swerveDrive: SwerveDrive):

        super().__init__()

        self.drive = swerveDrive
        self.timer = Timer()

        self.addRequirements([self.drive])

        self.pidController = PIDController(kChargeP, kChargeI, kChargeD)
        self.onChargeStation = False

    def initialize(self):

        self.onChargeStation = False
        self.timer.stop()
        self.timer.reset()

    def execute(self):

        SmartDashboard.putNumber("Yaw", self.drive.getYaw())
        SmartDashboard.putNumber("Pitch", self.drive.getPitch())

        if self.drive.getPitch() <= 7.5 and not self.onChargeStation:

            self.drive.translate(0, 0.3)
            SmartDashboard.putString("Auto Status", "Driving to Station")

        elif self.timer.get() <= 1:

            self.onChargeStation = True
            self.timer.start()

        else:

            power = (self.pidController.calculate(self.drive.getPitch(), 0.0))

            SmartDashboard.putString("Auto Status", "PID Control")

            if abs(power) <= 0.5:

                self.drive.translate(0, power)

        SmartDashboard.putBoolean("Running", True)

    def end(self, interrupted: bool):

        self.drive.translate(0, 0)
        SmartDashboard.putBoolean("Running", False)

    def isFinished(self):

        return False