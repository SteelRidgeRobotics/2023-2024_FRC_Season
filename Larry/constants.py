from pathplannerlib import PathConstraints
"""
CONSTANTS
"""

# Options
kUsingGuitarController = False
kDebug = False # Turns off some temporary SmartDashboard values (basically if we dont want to clutter up things ;))

# Larry stats (for ease of access and path tracing) (in meters per second, m/s)
klarryMaxSpeed = 3.658 # m/s
klarryMaxRotSpeed = 10.47197551198692 # rads/s
#klarryMaxAcceleration = # m/s^2
klarryWheelSize = 4 / 12 * 3.28

# Wheels
kmaxWheelSpeed = 1000 # in TalonFX native units (PLACEHOLDER)

# PathPlanner
kdefaultPathConstraints = PathConstraints(4.0, 3.0)

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

# Odometry and Kinematics
krobotSize = 0.381

# Controllers
kdriverControllerPort = 0
kdeadband = 0.15

# Encoders
ktimeoutMs = 10
kF = 0.05282272  # Feed forward
kP = 0.6  # Proportional
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
