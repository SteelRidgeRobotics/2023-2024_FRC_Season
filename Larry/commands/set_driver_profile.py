import commands2
from enum import Enum
from subsystems.swerve_drive import SwerveDrive

class BumperOptions(Enum):
    NONE = 0
    SPEED = 1
    ROTATION = 2
    TRANSLATION = 3

class TriggerOptions(Enum):
    NONE = 0
    ANALOG_0_2 = 2
    ANALOG_0_1 = 1
    ANALOG_0_05 = 0.5
    ANALOG_0_025 = 0.25
    SET_2 = 2
    SET_05 = 0.5
    SET_025 = 0.25


# DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
# "kLeftBumperOption": BumperOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": BumperOptions.SPEED, "kRightBumperFactor": 0.5, 
# "kLeftTriggerOption": TriggerOptions.NONE, "kRightTriggerOption": TriggerOptions.NONE}

class DriverProfiles(Enum):
    DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
               "kLeftBumperOption": BumperOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": BumperOptions.SPEED, "kRightBumperFactor": 0.5, 
               "kLeftTriggerOption": TriggerOptions.NONE, "kRightTriggerOption": TriggerOptions.NONE}
    
    DEFAULT_SLOW_BUMPER_SPEEDUP = {"defSpeedMultiplier": 0.25, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, "kBumperFactor": 2.0}

class SetDriverProfile(commands2.CommandBase):

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
