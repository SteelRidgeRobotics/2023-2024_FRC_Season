from wpilib import SmartDashboard
from wpilib.event import BooleanEvent, EventLoop
from wpilib.interfaces import GenericHID

"""
guitar.py

Created by Pickle_Face5

Version 1.0.0
"""


class Guitar(GenericHID):
    """
    Handle input from guitar controllers connected to the Driver Station.

    This class handles guitar input that comes from the Driver Station. Each time a value is requested the most recent
    value is returned. There is a single class instance for each controller and the mapping of ports to hardware buttons
     depends on the code in the Driver Station.
    """

    def __init__(self, port: int) -> None:
        super().__init__(port)

    class Axis:
        kJoyX = 0
        kJoyY = 1
        kWhamBar = 2
        kSlider = 4

    class Button:
        kGreen = 1
        kRed = 2
        kYellow = 3
        kBlue = 4
        kOrange = 5
        kStrumDown = 6
        kVol = 7
        kStar = 8
        kStrumUp = 9

    def sendValuesToSmartDashboard(self) -> None:  # Renamed 'self' to 's' to keep this function readable
        """
        Adds all button and axis values to Smart Dashboard.
        """
        SmartDashboard.putNumberArray("Joystick XY", [self.getJoystickX(), self.getJoystickY()])
        SmartDashboard.putBooleanArray("Fret Buttons",
                                       [self.getGreenButton(), self.getRedButton(), self.getYellowButton(),
                                        self.getBlueButton(), self.getOrangeButton()])
        SmartDashboard.putBooleanArray("Strum Bar (Up, Down)", [self.getStrumBarUp(), self.getStrumBarDown()])
        SmartDashboard.putNumber("Slider", self.getSlider())
        SmartDashboard.putNumber("Whammy Bar", self.getWhammyBar())

    def getButton(self, id) -> bool:
        """
        Returns a buttons value with the given id. Button indexes start at 1.

        This is mainly used as a helper function for the other button methods, but can be used for any externally added
        buttons.
        """
        return super().getRawButton(id)

    def getButtonPressed(self, id) -> bool:
        """
        Returns true if the button was pressed since the last check. Button indexes start at 1.

        This is mainly used as a helper function for the other button methods, but can be used for any externally added
        buttons.
        """
        return super().getRawButtonPressed(id)

    def getButtonReleased(self, id) -> bool:
        """
        Returns true if the button was released since the last check. Button indexes start at 1.

        This is mainly used as a helper function for the other button methods, but can be used for any externally added
        buttons.
        """
        return super().getRawButtonReleased(id)

    def getGreenButton(self) -> bool:
        """
        Returns true if the Green Fret button is being pressed.
        """
        return self.getButton(self.Button.kGreen)

    def getGreenButtonPressed(self) -> bool:
        """
        Returns true if the Green Fret button was pressed since the last check.
        """
        return self.getButtonPressed(self.Button.kGreen)

    def getGreenButtonReleased(self) -> bool:
        """
        Returns true if the Green Fret button was released since the last check.
        """
        return self.getButtonReleased(self.Button.kGreen)

    def greenButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Green Fret button's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getGreenButton)

    def getRedButton(self) -> bool:
        """
        Returns true if the Red Fret button is being pressed.
        """
        return self.getButton(self.Button.kRed)

    def getRedButtonPressed(self) -> bool:
        """
        Returns true if the Red Fret button was pressed since the last check.
        """
        return self.getButtonPressed(self.Button.kRed)

    def getRedButtonReleased(self) -> bool:
        """
        Returns true if the Red Fret button was released since the last check.
        """
        return self.getButtonReleased(self.Button.kRed)

    def redButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Red Fret button's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getRedButton)

    def getYellowButton(self) -> bool:
        """
        Returns true if the Yellow Fret button is being pressed.
        """
        return self.getButton(self.Button.kYellow)

    def getYellowButtonPressed(self) -> bool:
        """
        Returns true if the Yellow Fret button was pressed since the last check.
        """
        return self.getButtonPressed(self.Button.kYellow)

    def getYellowButtonReleased(self) -> bool:
        """
        Returns true if the Yellow Fret button was released since the last check.
        """
        return self.getButtonReleased(self.Button.kYellow)

    def yellowButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Yellow Fret button's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getYellowButton)

    def getBlueButton(self) -> bool:
        """
        Returns true if the Blue Fret button is being pressed.
        """
        return self.getButton(self.Button.kBlue)

    def getBlueButtonPressed(self) -> bool:
        """
        Returns true if the Blue Fret button was pressed since the last check.
        """
        return self.getButtonPressed(self.Button.kBlue)

    def getBlueButtonReleased(self) -> bool:
        """
        Returns true if the Blue Fret button was released since the last check.
        """
        return self.getButtonReleased(self.Button.kBlue)

    def blueButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Blue Fret button's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getBlueButton)

    def getOrangeButton(self) -> bool:
        """
        Returns true if the Orange Fret button is being pressed.
        """
        return self.getButton(self.Button.kOrange)

    def getOrangeButtonPressed(self) -> bool:
        """
        Returns true if the Orange Fret button was pressed since the last check.
        """
        return self.getButtonPressed(self.Button.kOrange)

    def getOrangeButtonReleased(self) -> bool:
        """
        Returns true if the Orange Fret button was released since the last check.
        """
        return self.getButtonReleased(self.Button.kOrange)

    def orangeButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Orange Fret button's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getOrangeButton)

    def getStrumBar(self) -> bool:
        """
        Returns true if the strum bar is being strummed/pressed up OR down.

        The strum bar is technically 2 buttons, but in case you want to treat it as 1 button (pushed either up or down)
        use this.
        """
        return self.getButton(self.Button.kStrumDown) or self.getButton(self.Button.kStrumUp)

    def getStrumBarPressed(self) -> bool:
        """
        Returns true if the strum bar was pressed up or down since the last check.

        If you want to check if it was pressed in a specific direction, use getStrumBarPressedUp or
        getStrumBarPressedDown.
        """
        return self.getButtonPressed(self.Button.kStrumDown) or self.getButtonPressed(self.Button.kStrumUp)

    def getStrumBarReleased(self) -> bool:
        """
        Returns true if the strum bar was released since the last check.

        If you want to check if it was released in a specific direction, use getStrumBarReleasedUp or
        getStrumBarReleasedDown.
        """
        return self.getButtonReleased(self.Button.kStrumDown) or self.getButtonReleased(self.Button.kStrumUp)

    def strumBar(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Strum Bar's digital signals.
        """
        return BooleanEvent(loop, lambda: self.getStrumBar)

    def getStrumBarUp(self) -> bool:
        """
        Returns true if the strum bar is being "strummed" upwards.

        Up is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButton(self.Button.kStrumUp)

    def getStrumBarUpPressed(self) -> bool:
        """
        Returns true if the strum bar has been "strummed" upwards since the last check.

        Up is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButtonPressed(self.Button.kStrumUp)

    def getStrumBarUpReleased(self) -> bool:
        """
        Returns true if the strum bar has been released from the upward position since the last check.

        Up is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButtonReleased(self.Button.kStrumUp)

    def strumBarUp(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Strum Bar's upwards digital signal.
        """
        return BooleanEvent(loop, lambda: self.getStrumBarUp)

    def getStrumBarDown(self) -> bool:
        """
        Returns true if the strum bar is being "strummed" downwards.

        Down is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButton(self.Button.kStrumDown)

    def getStrumBarDownPressed(self) -> bool:
        """
        Returns true if the strum bar has been "strummed" downwards since the last check.

        Up is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButtonPressed(self.Button.kStrumDown)

    def getStrumBarDownReleased(self) -> bool:
        """
        Returns true if the strum bar has been released from the downward position since the last check.

        Up is considered when you're facing the guitar and holding the guitar pointing right.
        """
        return self.getButtonReleased(self.Button.kStrumDown)

    def strumBarDown(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Strum Bar's downwards digital signal.
        """
        return BooleanEvent(loop, lambda: self.getStrumBarDown)

    def getVolumeButtons(self) -> bool:
        """
        Returns true if either volume button is being pressed.

        Both volume buttons are recognized as 1 button in Driver Station.
        """
        return self.getButton(self.Button.kVol)

    def getVolumeButtonsPressed(self) -> bool:
        """
        Returns true if either volume button has been pressed since the last check.

        Both volume buttons are recognized as 1 button in Driver Station.
        """
        return self.getButtonPressed(self.Button.kVol)

    def getVolumeButtonsReleased(self) -> bool:
        """
        Returns true if either volume button has been released since the last check.

        Both volume buttons are recognized as 1 button in Driver Station.
        """
        return self.getButtonReleased(self.Button.kVol)

    def volumeButtons(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Volume buttons's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getVolumeButtons)

    def getStarPowerButton(self) -> bool:
        """
        Returns true if the Star Power button is being pressed.

        The Star Power button is located in between the volume buttons. It also says "Star Power" on it.
        """
        return self.getButton(self.Button.kStar)

    def getStarPowerButtonPressed(self) -> bool:
        """
        Returns true if the Star Power button has been pressed since the last check.

        The Star Power button is located in between the volume buttons. It also says "Star Power" on it.
        """
        return self.getButtonPressed(self.Button.kStar)

    def getStarPowerButtonReleased(self) -> bool:
        """
        Returns true if the Star Power button has been released since the last check.

        The Star Power button is located in between the volume buttons. It also says "Star Power" on it.
        """
        return self.getButtonReleased(self.Button.kStar)

    def starPowerButton(self, loop: EventLoop) -> BooleanEvent:
        """
        Constructs an event instance around the Star Power buttons's digital signal.
        """
        return BooleanEvent(loop, lambda: self.getStarPowerButton)

    def getAxis(self, id) -> float:
        """
        Returns the current value of an axis with the given id.

        This should only be used if an axis was externally added to the guitar, since all axes have their own methods.
        """
        return super().getRawAxis(id)

    def getJoystickX(self) -> float:
        """
        Returns the current X value of the joystick.

        Minimum value (may differ from guitar to guitar) is -0.75, maximum value is 0.81. (Values may differ between
        guitars)
        """
        return self.getAxis(self.Axis.kJoyX)

    def getJoystickY(self) -> float:
        """
        Returns the current Y value of the joystick.

        Minimum value (may differ from guitar to guitar) is -0.94, maximum value is 0.78. (Values may differ between
        guitars)
        """
        return self.getAxis(self.Axis.kJoyY)

    def getWhammyBar(self) -> float:
        """
        Returns the current Whammy Bar rotation.

        The Whammy Bar only moves in 1 axis (recognized in Driver Station as Axis 2)
        """
        return self.getAxis(self.Axis.kWhamBar)

    def getSlider(self) -> float:
        """
        Returns current value of the slider (located beneath the Orange Fret button).

        Values range from -0.94 - 0.73.
        """
        return self.getAxis(self.Axis.kSlider)