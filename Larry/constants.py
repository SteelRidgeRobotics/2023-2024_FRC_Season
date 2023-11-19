from enum import Enum

"""
CONSTANTS
"""

# Options
kUsingGuitarController = False
kDebug = True # Turns off some temporary SmartDashboard values (basically if we dont want to clutter up things ;))

# Multipliers
kDefaultSpeedMultplier = 1.0
kDefaultTranslationMultiplier = 1.0
kDefaultRotationMultiplier = 1.0

# Driving Profile Enums
class MultiplierOptions(Enum):
    NONE = 0
    SPEED = 1
    ROTATION = 2
    TRANSLATION = 3

class TriggerOptions(Enum):
    NONE = 0
    ANALOG_1_2 = 2
    ANALOG_0_1 = 1
    ANALOG_0_05 = 0.5
    ANALOG_0_025 = 0.25
    SET_2 = 2
    SET_05 = 0.5
    SET_025 = 0.25

# Motors
kleftFrontSpeedID = 0
kleftRearSpeedID = 1
krightFrontSpeedID = 2
krightRearSpeedID = 3

kleftFrontDirectionID = 4
kleftRearDirectionID = 5
krightFrontDirectionID = 6
krightRearDirectionID = 7

# Controllers
kdriverControllerPort = 0
kdeadband = 0.15

# Encoders
ktimeoutMs = 10
kF = 0.05282272  # Feed forward
kP = 0.3  # Proportional
kI = 0.004  # Integral
kD = 2  # Derivative
kIzone = 150
kcruiseVel = 21134.0  # Cruise Velocity at 100% of max (max = 21134)
kcruiseAccel = 21134.0  # Cruise Acceleration same as velocity
kSlotIdx = 0
kPIDLoopIdx = 0

# CANCoders
kflCANcoderID = 10
krlCANcoderID = 11
kfrCANcoderID = 12
krrCANcoderID = 13

# Offsets
kflCANoffset = 350.5078125
krlCANoffset = 179.12109375
kfrCANoffset = 324.755859375
krrCANoffset = 28.828125

ksteeringGearRatio = 150 / 7

# PID constants for the charge station
kChargeP = 0.0115
kChargeI = 0.0
kChargeD = 0.0013
