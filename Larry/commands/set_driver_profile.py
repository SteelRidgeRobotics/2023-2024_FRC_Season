import commands2
from enum import Enum
from subsystems.swerve_drive import SwerveDrive

class DriverProfiles(Enum):
    DEFAULT = {"defSpeedMultiplier": 1.0, "defTranslationMultiplier": 1.0, "defRotationMultiplier": 1.0, "kBumperFactor": 0.5}
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
        self.swerveDrive.setBumperSlowdownFactor(selectedProfile.get("kBumperFactor"))
