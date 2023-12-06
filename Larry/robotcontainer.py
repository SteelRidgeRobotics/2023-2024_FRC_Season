from commands2 import Command, SubsystemBase
from constants import *
from commands.drive_with_controller import DriveControllerDefault, DriveControllerDefaultSlow, DriveControllerWyatt
from commands.drive_with_guitar import DriveWithGuitar
from commands.charge_station import ChargeStation
from commands.follow_path import FollowPath
from frc6343.controller.guitar.guitar import Guitar
from subsystems.swerve_drive import SwerveDrive
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
        self.autoChooser.addOption("Charge Station", ChargeStation(self.swerveDrive))
        self.autoChooser.addOption("PathPlanner Test 1", FollowPath(self.swerveDrive, "Test 1"))

        SmartDashboard.putData("Auto", self.autoChooser)

        # profile chooser
        self.profileChooser = SendableChooser()
        self.profileChooser.setDefaultOption("Default", DriveControllerDefault(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getBackButton(), lambda: self.driverController.getStartButton()))
        self.profileChooser.addOption("Default Slow", DriveControllerDefaultSlow(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getBackButton(), lambda: self.driverController.getStartButton()))
        self.profileChooser.addOption("Wyatt", DriveControllerWyatt(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getLeftTriggerAxis(), lambda: self.driverController.getRightTriggerAxis()))

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
    
    def getSwerveDrive(self) -> SubsystemBase:
        return self.swerveDrive