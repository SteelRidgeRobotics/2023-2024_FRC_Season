import commands2
from enum import Enum
from constants import MultiplierOptions, TriggerOptions
from subsystems.swerve_drive import SwerveDrive

# DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
# "kLeftBumperOption": BumperOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": BumperOptions.SPEED, "kRightBumperFactor": 0.5, 
# "kLeftTriggerOption": TriggerOptions.NONE, "kRightTriggerOption": TriggerOptions.NONE}

class DriverProfiles(Enum):
    DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, 
               "kLeftBumperOption": MultiplierOptions.SPEED, "kLeftBumperFactor": 0.5, "kRightBumperOption": MultiplierOptions.SPEED, "kRightBumperFactor": 0.5, 
               "kLeftTriggerOption": TriggerOptions.NONE, "kLeftTriggerMode": MultiplierOptions.NONE, 
               "kRightTriggerOption": TriggerOptions.NONE, "kRightTriggerMode": MultiplierOptions.NONE}
    
    DEFAULT_SLOW_BUMPER_SPEEDUP = {"defSpeedMultiplier": 0.25, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0,
                                   "kLeftBumperOption": MultiplierOptions.SPEED, "kLeftBumperFactor": 2.0, "kRightBumperOption": MultiplierOptions.SPEED, "kRightBumperFactor": 2.0,
                                   "kLeftTriggerOption": TriggerOptions.NONE, "kLeftTriggerMode": MultiplierOptions.NONE, 
                                    "kRightTriggerOption": TriggerOptions.NONE, "kRightTriggerMode": MultiplierOptions.NONE}

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
