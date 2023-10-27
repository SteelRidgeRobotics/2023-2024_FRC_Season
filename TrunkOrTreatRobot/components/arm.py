import wpilib
import constants
import ctre

class Arm:

    armMotor: ctre.TalonSRX

    def setup(self):

        self.armMotor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self, guitarJoystickY):

        self.left = guitarJoystickY

        if abs(self.left) <= constants.DEADBAND:

            self.left = 0

    def execute(self):

        self.armMotor.set(ctre.TalonSRXControlMode.PercentOutput, -(self.left))
        wpilib.SmartDashboard.putNumber("MotorValue", -(self.left))
        wpilib.SmartDashboard.putNumber("MotorActualValue", self.armMotor.getMotorOutputPercent())





    

    
