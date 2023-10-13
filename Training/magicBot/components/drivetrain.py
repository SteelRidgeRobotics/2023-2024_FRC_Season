import constants
import ctre
import magicbot

class Drivetrain:

    FLMotor: ctre.TalonFX
    BLMotor: ctre.TalonFX
    FRMotor: ctre.TalonFX
    BRMotor: ctre.TalonFX

    left = magicbot.will_reset_to(0)
    right = magicbot.will_reset_to(0)

    def setup(self):

        self.BLMotor.follow(self.FLMotor)
        self.BRMotor.follow(self.FRMotor)

        self.FRMotor.setInverted(True)
        self.BRMotor.setInverted(True)

        self.FLMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.FRMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.BLMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.BRMotor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self, leftJoy, rightJoy):

        self.left = leftJoy
        self.right = rightJoy

        if abs(self.left) <= constants.DEADBAND:
            self.left = 0

        if abs(self.right) <= constants.DEADBAND:
            self.right = 0
    
    def execute(self):

        self.FLMotor.set(ctre.TalonFXControlMode.PercentOutput, -self.left)
        self.FRMotor.set(ctre.TalonFXControlMode.PercentOutput, -self.right)