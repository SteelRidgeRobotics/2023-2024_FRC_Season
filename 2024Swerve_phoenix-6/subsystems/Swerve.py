from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands2.subsystem import *
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry,SwerveModulePosition, SwerveModuleState
import navx
from Swervemodule import *
from SwerveModuleConstants import *
from Constants import *
from wpilib import SmartDashboard

class Swerve(Subsystem):
    def Swerve(self):
        self.gyro = navx.AHRS.create_spi()
        #self.gyro = Pigeon2(Constants.Swerve.pigeonID)
        #self.gyro.configurator.apply(Pigeon2Configuration())
        self.gyro.reset()
        #self.gyro.getYaw()

        self.mSwerveMods = [
            SwerveModule(0, Constants.Swerve.Mod0.constants),
            SwerveModule(1, Constants.Swerve.Mod1.constants),
            SwerveModule(2, Constants.Swerve.Mod2.constants),
            SwerveModule(3, Constants.Swerve.Mod3.constants)
            
        ]

        self.swerveOdometry = SwerveDrive4Odometry(
            Constants.Swerve.swerveKinematics, self.getGyroYaw(), self.getModulePositions()
        )

    def drive(self, translation:Translation2d, rotation:float, field_relative:bool, is_open_loop:bool):
        swerveModuleStates = (
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.X(), 
                    translation.Y(),
                    rotation,
                    self.getHeading()
                ) if field_relative else ChassisSpeeds(
                    translation.X(),
                    translation.Y(),
                    rotation
                )
            )
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed)

        for mod in self.mSwerveMods:
            
            mod.set_desired_state(swerveModuleStates[mod.moduleNumber], is_open_loop)

    def setModuleStates(self, desired_states:SwerveModuleState):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, Constants.Swerve.maxSpeed)

        for mod in self.mSwerveMods:
            mod.set_desired_state(desired_states[mod.moduleNumber], False)

    def getModuleStates(self, states:SwerveModuleState):
        states = SwerveModuleState[4]
        states = [mod.get_state() for mod in self.mSwerveMods]
        return states

    def getModulePositions(self, positions:SwerveModulePosition):
        positions = SwerveModulePosition[4]
        positions = [mod.get_position() for mod in self.mSwerveMods]
        return positions

    def getPose(self):
        return self.swerveOdometry.getPose()

    def setPose(self, pose:Pose2d):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), self.getModulePositions(), pose)

    def getHeading(self):
        return self.getPose().rotation()

    def setHeading(self, heading:Rotation2d):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), self.getModulePositions(), Pose2d(self.getPose().translation(), heading))

    def zeroHeading(self):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), self.getModulePositions(), Pose2d(self.getPose().translation(), Rotation2d()))

    def getGyroYaw(self):
        return Rotation2d.fromDegrees(self.gyro.getYaw())

    def resetModulesToAbsolute(self):
        for mod in self.mSwerveMods:
            mod.reset_to_absolute()

    def periodic(self):
        self.swerveOdometry.update(self.getGyroYaw(), self.getModulePositions())

        for mod in self.mSwerveMods:
            SmartDashboard.putNumber("Mod " + str(mod.moduleNumber) + " CANcoder", mod.get_cancoder().degrees())
            SmartDashboard.putNumber("Mod " + str(mod.moduleNumber) + " Angle", mod.get_position().angle.degrees())
            SmartDashboard.putNumber("Mod " + str(mod.moduleNumber) + " Velocity", mod.get_state().speed)
