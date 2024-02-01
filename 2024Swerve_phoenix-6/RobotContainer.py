from wpilib import Joystick
from commands2.button import JoystickButton
from commands2 import InstantCommand
from commands.TeleopSwerve import *
from autos.exampleAuto import *

class RobotContainer:
    def __init__(self):
        # Controllers
        self.driver = Joystick(0)

        # Drive Controls
        self.translationAxis = 1  # Replace with the correct axis value
        self.strafeAxis = 0  # Replace with the correct axis value
        self.rotationAxis = 4  # Replace with the correct axis value

        # Driver Buttons
        self.zeroGyro = JoystickButton(self.driver, 4)  # Replace with the correct button value
        self.robotCentric = JoystickButton(self.driver, 5)  # Replace with the correct button value

        # Subsystems
        self.s_Swerve = Swerve()
        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                lambda: -self.driver.getRawAxis(self.translationAxis),
                lambda: -self.driver.getRawAxis(self.strafeAxis),
                lambda: -self.driver.getRawAxis(self.rotationAxis),
                lambda: self.robotCentric.getAsBoolean()
            )
        )

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self):
        # Driver Buttons
        self.zeroGyro.whileTrue(InstantCommand(self.s_Swerve.zeroHeading()))

    def get_autonomous_command(self):
        # An ExampleAuto command will run in autonomous
        return ExampleAuto(self.s_Swerve)
