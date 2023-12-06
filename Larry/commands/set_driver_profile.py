from commands2 import CommandBase
from enum import Enum
from constants import MultiplierOptions, TriggerOptions
from subsystems.swerve_drive import SwerveDrive

# DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
# "kLeftBumperOption": BumperOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": BumperOptions.SPEED, "kRightBumperFactor": 0.5, 
# "kLeftTriggerOption": TriggerOptions.NONE, "kRightTriggerOption": TriggerOptions.NONE}

"""
Profile Options:

A. TRANSLATING (probably tuple) | Def: (True, False, False)
    1. Left Joystick
    2. Right Joystick
    3. DPad

B. ROTATING (tuple) | Def: (False, True, False, False)
    1. Left Joystick
    2. Right Joystick
    3. DPad
    4. L/R Bumpers

C. MULTIPLIER MANIPS (Good luck)
    a. Toggles
        1. A,B,X,Y Buttons
        2. LR Bumpers
        3. LR Triggers
        4. Back Buttons (may not work, due to pain to sim e.g i don't have them so it's on the "maybe" list.)
        5: Up Down Left Right D-PAD
    b. Analog
        1. LR Triggers
    c. Dialable (aka press multiple times to increase, press multiple times to decrease)
        1. DPad
        2. LR Bumpers
        3. LR Triggers

D. MULTIPLIER FACTORS (how much we change them when ____)
    a. Set Modes
        1. When pressed, x. When released, y
        2. Toggle to x or y
        3. Toggle increase by z, limit x <= z <= y
    b. Analog (Triggers)
        1. When pressed, slowly go from x to y
        2. When pressed, set to x. Else, set to y
    c. Misc
        1. If button 1 pressed, x, if both pressed, y, else z
"""

class DriverProfiles(Enum):
    DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
               "kLeftBumperOption": MultiplierOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": MultiplierOptions.SPEED, "kRightBumperFactor": 0.5, 
               "kLeftTriggerOption": TriggerOptions.NONE, "kLeftTriggerMode": MultiplierOptions.NONE, 
               "kRightTriggerOption": TriggerOptions.NONE, "kRightTriggerMode": MultiplierOptions.NONE}
    
    DEFAULT_SLOW_BUMPER_SPEEDUP = {"defSpeedMultiplier": 0.25, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0,
                                   "kLeftBumperOption": MultiplierOptions.SPEED, "kLeftBumperFactor": 2.0, "kRightBumperOption": MultiplierOptions.SPEED, "kRightBumperFactor": 2.0,
                                   "kLeftTriggerOption": TriggerOptions.NONE, "kLeftTriggerMode": MultiplierOptions.NONE, 
                                    "kRightTriggerOption": TriggerOptions.NONE, "kRightTriggerMode": MultiplierOptions.NONE}

class SetDriverProfile(CommandBase):

    def __init__(self, swerveDrive: SwerveDrive, driverProfile: DriverProfiles) -> None:
        super().__init__()

        self.swerveDrive = swerveDrive
        self.driverProfile = driverProfile

    def initialize(self) -> None:
        selectedProfile = None

        for profile in DriverProfiles:
            if profile.name == self.driverProfile.name:
                selectedProfile = profile
                break

        if selectedProfile is None:
            selectedProfile = DriverProfiles.DEFAULT

        # Turn enum into dict
        selectedProfile = dict(selectedProfile.value)

        self.swerveDrive.setDefaultSpeedMultiplier(selectedProfile.get("defSpeedMultiplier"))
        self.swerveDrive.setDefaultTranslationMultiplier(selectedProfile.get("defTranslationMultiplier"))
        self.swerveDrive.setDefaultRotationMultiplier(selectedProfile.get("defRotationMultiplier"))

        if selectedProfile.get("kLeftBumperOption") != MultiplierOptions.NONE:
            self.swerveDrive.setLeftBumperMode(selectedProfile.get("kLeftBumperOption"))
            self.swerveDrive.setLeftBumperFactor(selectedProfile.get("kLeftBumperFactor"))

        if selectedProfile.get("kRightBumperOption") != MultiplierOptions.NONE:
            self.swerveDrive.setRightBumperMode(selectedProfile.get("kRightBumperOption"))
            self.swerveDrive.setRightBumperFactor(selectedProfile.get("kRightBumperFactor"))

        if selectedProfile.get("kLeftTriggerOption") != TriggerOptions.NONE:
            self.swerveDrive.setLeftTriggerMode(selectedProfile.get("kLeftTriggerMode"))
            self.swerveDrive.setLeftTriggerOption(selectedProfile.get("kLeftTriggerOption"))

        if selectedProfile.get("kRightTriggerOption") != TriggerOptions.NONE:
            self.swerveDrive.setRightTriggerMode(selectedProfile.get("kRightTriggerMode"))
            self.swerveDrive.setRightTriggerOption(selectedProfile.get("kRightTriggerOption"))
