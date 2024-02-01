from wpimath.geometry import Rotation2d


class SwerveModuleConstants:
    """
    def __init__(self, drive_motor_id:int, angle_motor_id:int, cancoder_id:int, angle_offset:Rotation2d):
        self.driveMotorID = drive_motor_id
        self.angleMotorID = angle_motor_id
        self.cancoderID = cancoder_id
        self.angleOffset = angle_offset
    """
    
    def SwerveModuleConstants(self,driveMotorID:int, angleMotorID:int, canCoderID:int, angleOffset:Rotation2d):
        self.driveMotorID = driveMotorID
        self.angleMotorID = angleMotorID
        self.cancoderID = canCoderID
        self.angleOffset = angleOffset
