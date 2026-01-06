from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


class Intake(Subsystem):
    def __init__(self, leaderCanID, leaderInverted=True, followerCanID=None, followerInverted=False) -> None:
        super().__init__()

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushed)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(leaderInverted)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)  # by default disabled (until `intakeGamepiece()`)
        self.motor.configure(self.motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushed)
            followerConfig = SparkBaseConfig()
            followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            self.followerMotor.configure(followerConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        # 3. safe initial state
        self._setSpeed(0.0)


    def enableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        # ^^ do not reset and do not persist, just enable the switch

    def disableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        # ^^ do not reset and do not persist, just disable the switch

    def isGamepieceInside(self) -> bool:
        return self.limitSwitch.get()

    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()

    def periodic(self):
        SmartDashboard.putBoolean("intakeFull", self.limitSwitch.get())

    def intakeGamepiece(self, speed=0.25):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self, speed=1.0, speed2=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25, speed2=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-speed)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25, speed2=None):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)

    def stop(self):
        self._setSpeed(0)
        print("Intake::stop")

    def _setSpeed(self, speed):
        self.motor.set(speed)
        SmartDashboard.putNumber("intakeSpeed", speed)
