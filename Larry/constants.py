# options
kUsingGuitarController = False
kDebug = True # Turns off some temporary SmartDashboard values (basically if we dont want to clutter up things ;))
kBumperSlowdownFactor = 0.5 # Amount the bumpers slowdown the robot. Default is 0.5, 50%

# multipliers
kDefaultSpeedMultplier = 1.0
kDefaultTranslationMultiplier = 1.0
kDefaultRotationMultiplier = 1.0

# motors
kleftFrontSpeedID = 0
kleftRearSpeedID = 1
krightFrontSpeedID = 2
krightRearSpeedID = 3

kleftFrontDirectionID = 4
kleftRearDirectionID = 5
krightFrontDirectionID = 6
krightRearDirectionID = 7

# controllers
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

# offsets
kflCANoffset = 350.5078125
krlCANoffset = 179.12109375
kfrCANoffset = 324.755859375
krrCANoffset = 28.828125

ksteeringGearRatio = 150 / 7

# PID constants for the charge station
kChargeP = 0.0115
kChargeI = 0.0
kChargeD = 0.0013
