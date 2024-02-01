from robot import *
import robot


from phoenix6.controls import DutyCycleOut, PositionVoltage, VelocityVoltage
from phoenix6.hardware import CANcoder, TalonFX
#from phoenix6.configs import *

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import Conversions
from Conversions import *
import SwerveModuleConstants

from Constants import *
from SwerveModuleConstants import *




class SwerveModule:
    def __init__(self):
        # Feedforward
        self.driveFeedForward = SimpleMotorFeedforwardMeters(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA)
        
        

        # Drive motor control requests
        self.driveDutyCycle = DutyCycleOut(0)
        self.driveVelocity = VelocityVoltage(0)

        # Angle motor control requests
        self.anglePosition = PositionVoltage(0)


        
    def SwerveModule(self):

        moduleConstants = SwerveModuleConstants()
        
        moduleNumber = int

        self.moduleNumber = moduleNumber 

        self.angleOffset = moduleConstants.angleOffset
        

        # Angle Encoder Config
        self.angleEncoder = CANcoder(moduleConstants.cancoderID)
        #self.angleEncoder.configFactoryDefault()
        self.angleEncoder.configurator.apply(Robot.ctre_configs.swerveCANcoderConfig)
        
        
        

        # Angle Motor Config
        self.mAngleMotor = TalonFX(moduleConstants.angleMotorID)
        self.mAngleMotor.configurator.apply(Robot.ctre_configs.swerveAngleFXConfig)
        self.reset_to_absolute()
        

        # Drive Motor Config
        self.mDriveMotor = TalonFX(moduleConstants.driveMotorID)
        self.mDriveMotor.configurator.apply(Robot.ctre_configs.swerveDriveFXConfig)
        self.mDriveMotor.configurator.set_position(0.0)
    



    def set_desired_state(self, desired_state:SwerveModuleState, is_open_loop:bool):
        desired_state = SwerveModuleState.optimize(desired_state, self.get_state().angle)
        self.mAngleMotor.set_control(self.anglePosition.with_position(desired_state.angle.fromDegrees()/360))
        self.set_speed(desired_state, is_open_loop)

    def set_speed(self, desired_state:SwerveModuleState, is_open_loop:bool):
        if is_open_loop:
            self.driveDutyCycle.output = desired_state.speed / Constants.Swerve.maxSpeed
            self.mDriveMotor.set(self.driveDutyCycle)
        else:
            self.driveVelocity.velocity = Conversions.MPS_to_RPS(desired_state.speed, Constants.Swerve.wheelCircumference)
            self.driveVelocity.feed_forward = self.driveFeedForward.calculate(desired_state.speed)
            self.mDriveMotor.set_control(self.driveVelocity)

    def get_cancoder(self)->Rotation2d:
        
        return Rotation2d.fromDegrees(360*self.angleEncoder.get_absolute_position().value)

    def reset_to_absolute(self)->None:
        absolute_position = self.get_cancoder().fromDegrees()/360-self.angleOffset.fromDegrees()/360
        self.mAngleMotor.set_position(absolute_position)

    def get_state(self)->SwerveModuleState:
        return SwerveModuleState(
            Conversions.RPS_to_MPS(self.mDriveMotor.get_velocity().value, Constants.Swerve.wheelCircumference),
            Rotation2d.fromDegrees(360*self.mAngleMotor.get_position().value)
        )

    def get_position(self)->SwerveModulePosition:
        return SwerveModulePosition(
            Conversions.rotations_to_meters(self.mDriveMotor.get_position().value, Constants.Swerve.wheelCircumference),
            Rotation2d.fromDegrees(360*self.mAngleMotor.get_position().value)
        )