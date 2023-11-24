"""
CONSTANTS (yay)
"""
# Options
kDebug = True
kdeadband = 0.15


# Multipliers
kDefaultSpeedMultplier = 1.0
kDefaultTranslationMultiplier = 1.0
kDefaultRotationMultiplier = 1.0


# Motors
kleftFrontSpeedID = 0
kleftRearSpeedID = 1
krightFrontSpeedID = 2
krightRearSpeedID = 3

kleftFrontDirectionID = 4
kleftRearDirectionID = 5
krightFrontDirectionID = 6
krightRearDirectionID = 7

ksteeringGearRatio = 150 / 7


# CANCoders
kflCANcoderID = 10
krlCANcoderID = 11
kfrCANcoderID = 12
krrCANcoderID = 13


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


# Offsets
kflCANoffset = 350.5078125
krlCANoffset = 179.12109375
kfrCANoffset = 324.755859375
krrCANoffset = 28.828125