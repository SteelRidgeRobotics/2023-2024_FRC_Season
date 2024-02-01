from phoenix6.signals import InvertedValue, NeutralModeValue, SensorDirectionValue

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import *
from COTSTalonFXSwerveConstants import *
from SwerveModuleConstants import *



#from Swervemodule import *

class Constants:
    stickDeadband = 0.1

    class Swerve:
        pigeonID = 1
               
        chosenModule = COTSTalonFXSwerveConstants(COTSTalonFXSwerveConstants_SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants_SDS.MK4i.driveRatios.L2))
        #chosenModule=COTSTalonFXSwerveConstants_SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants_SDS.MK4i.driveRatios.L2)
        

        # Drivetrain Constants
        trackWidth = inchesToMeters(21.73)
        wheelBase = inchesToMeters(21.73)
        wheelCircumference = chosenModule.wheel_circumference

        # Swerve Kinematics
        swerveKinematics = SwerveDrive4Kinematics(
            Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        )

        # Module Gear Ratios
        driveGearRatio = chosenModule.drive_gear_ratio
        angleGearRatio = chosenModule.angle_gear_ratio

        # Motor Inverts
        angleMotorInvert = COTSTalonFXSwerveConstants(angle_motor_invert=chosenModule.angle_motor_invert)
        #angleMotorInvert = chosenModule.angle_motor_invert
        #driveMotorInvert = InvertedValue
        #driveMotoInvert  = chosenModule.drive_motor_invert
        driveMotorInvert = COTSTalonFXSwerveConstants(drive_motor_invert=chosenModule.drive_motor_invert)

        # Angle Encoder Invert
        #cancoderInvert = SensorDirectionValue
        #cancoderInvert = chosenModule.cancoder_invert
        cancoderInvert = COTSTalonFXSwerveConstants(cancoder_invert=chosenModule.cancoder_invert)

        # Swerve Current Limiting
        angleCurrentLimit = 25
        angleCurrentThreshold = 40
        angleCurrentThresholdTime = 0.1
        angleEnableCurrentLimit = True

        driveCurrentLimit = 35
        driveCurrentThreshold = 60
        driveCurrentThresholdTime = 0.1
        driveEnableCurrentLimit = True

        # Ramp Rates
        openLoopRamp = 0.25
        closedLoopRamp = 0.0

        # Angle Motor PID Values
        angleKP = chosenModule.angle_KP
        angleKI = chosenModule.angle_KI
        angleKD = chosenModule.angle_KD

        # Drive Motor PID Values
        driveKP = 0.12
        driveKI = 0.0
        driveKD = 0.0
        driveKF = 0.0

        # Drive Motor Characterization Values From SYSID
        driveKS = 0.32
        driveKV = 1.51
        driveKA = 0.27

        # Swerve Profiling Values
        maxSpeed = 4.5
        maxAngularVelocity = 10.0

        # Neutral Modes
        angleNeutralMode = NeutralModeValue.COAST
        driveNeutralMode = NeutralModeValue.BRAKE

        class Mod0:
            driveMotorID = 1
            angleMotorID = 2
            canCoderID = 1
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        class Mod1:
            driveMotorID = 3
            angleMotorID = 4
            canCoderID = 2
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        class Mod2:
            driveMotorID = 5
            angleMotorID = 6
            canCoderID = 3
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        class Mod3:
            driveMotorID = 7
            angleMotorID = 8
            canCoderID = 4
            angleOffset = Rotation2d.fromDegrees(0.0)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

    class AutoConstants:
        kMaxSpeedMetersPerSecond = 3
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularSpeedRadiansPerSecond = degreesToRadians(180)
        kMaxAngularSpeedRadiansPerSecondSquared = degreesToRadians(180)

        kPXController = 1
        kPYController = 1
        kPThetaController = 1

        kThetaControllerConstraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        )