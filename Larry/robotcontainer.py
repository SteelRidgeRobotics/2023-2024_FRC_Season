from commands2 import Command
from commands.autonomous import *
from commands.charge_station import ChargeStation
from commands.drive_with_guitar import DriveWithGuitar
from commands.driving_profiles import (DriveControllerDefault,
                                       DriveControllerDefaultSlow,
                                       DriveControllerCaden,
                                       DriveControllerWyatt,
                                       DriveWithKinematics)
from constants import *
from frc6343.controller.guitar.guitar import Guitar
from subsystems.swerve_drive import SwerveDrive
from autos.tests import *
from wpilib import SendableChooser, SmartDashboard, Timer, XboxController


class RobotContainer:
    def __init__(self) -> None:
        # init controllers
        self.driverController = XboxController(kdriverControllerPort) if not kUsingGuitarController else Guitar(kdriverControllerPort)

        self.timer = Timer

        # init subsystems
        self.swerveDrive = SwerveDrive()

        # auto chooser
        self.autoChooser = SendableChooser()

        # Add commands to auto command chooser
        self.autoChooser.setDefaultOption("Charge Station", ChargeStation(self.swerveDrive))
        self.autoChooser.addOption("Move Forward 1 (NEW)", TestForward(self.swerveDrive))
        self.autoChooser.addOption("Move Foward (NOT PP)", TestForwardNoPP(self.swerveDrive))

        SmartDashboard.putData("Auto", self.autoChooser)

        # profile chooser
        self.profileChooser = SendableChooser()
        self.profileChooser.setDefaultOption("Default", DriveControllerDefault(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getBackButton(), lambda: self.driverController.getStartButton()))
        self.profileChooser.addOption("Default Slow", DriveControllerDefaultSlow(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getBackButton(), lambda: self.driverController.getStartButton()))
        self.profileChooser.addOption("Wyatt", DriveControllerWyatt(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getLeftTriggerAxis(), lambda: self.driverController.getRightTriggerAxis()))
        self.profileChooser.addOption("Caden", DriveControllerCaden(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getLeftTriggerAxis(), lambda: self.driverController.getRightTriggerAxis()))
        self.profileChooser.addOption("KINEMATICS TEST", DriveWithKinematics(self.swerveDrive, lambda: self.driverController, True))
        self.profileChooser.addOption("KINEMATICS TEST (non-field relative)", DriveWithKinematics(self.swerveDrive, lambda: self.driverController, False))

        SmartDashboard.putData("Profile", self.profileChooser)

        self.configureButtonBindings()

        if kUsingGuitarController:
            self.swerveDrive.setDefaultCommand(
                DriveWithGuitar(self.swerveDrive,
                                lambda: self.driverController.getGreenButtonPressed(),
                                lambda: self.driverController.getRedButtonPressed(),
                                lambda: self.driverController.getYellowButtonPressed(),
                                lambda: self.driverController.getBlueButtonPressed(),
                                lambda: self.driverController.getStrumBarDownPressed(),
                                lambda: self.driverController.getStrumBarUpPressed()
                )
            )

    def configureButtonBindings(self):
        """This is where our trigger bindings for commands go"""

    def getAutonomousCommand(self) -> Command:
        return self.autoChooser.getSelected()
    
    def getDrivingMode(self) -> Command:
        return self.profileChooser.getSelected()
    
    def getSwerveDrive(self) -> SwerveDrive:
        return self.swerveDrive