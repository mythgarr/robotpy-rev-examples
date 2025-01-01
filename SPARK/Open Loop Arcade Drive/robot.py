from wpilib import TimedRobot, XboxController, SmartDashboard
import wpilib.drive
import wpimath.filter
from rev import SparkBaseConfig, SparkMaxConfig, SparkMax, SparkBase

from wpilib.simulation import DifferentialDrivetrainSim
from rev import SparkMaxSim, SparkSim
from wpimath.system.plant import DCMotor
from wpimath import units


kDeadzone = 0.1

def applyDeadzone(v):
  if v < -kDeadzone:
    return max(-1.0, (v + kDeadzone)/(1.0 - 2 * kDeadzone))
  if v > kDeadzone:
    return min(1.0, (v - kDeadzone)/(1.0 - 2 * kDeadzone))
  return 0.0

class MyRobot(TimedRobot):
  def __init__(self):
    '''
    Create new SPARK MAX configuration objects. These will store the
    configuration parameters for the SPARK MAXes that we will set below.
    '''
    TimedRobot.__init__(self)
    # Initialize the SPARKs
    self._leftLeader = SparkMax(1, SparkMax.MotorType.kBrushless)
    self._leftFollower = SparkMax(2, SparkMax.MotorType.kBrushless)
    self._rightLeader = SparkMax(3, SparkMax.MotorType.kBrushless)
    self._rightFollower = SparkMax(4, SparkMax.MotorType.kBrushless)
    
    # Create new SPARK MAX configuration objects. These will store the
    # configuration parameters for the SPARK MAXes that we will set below.
    globalConfig = SparkMaxConfig()
    rightLeaderConfig = SparkMaxConfig()
    rightFollowConfig = SparkMaxConfig()
    leftFollowerConfig = SparkMaxConfig()

    # Set parameters that will apply to all SPARKs. We will also use this as
    # the left leader config.
    (globalConfig.smartCurrentLimit(50)
     .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
     )
    
    # Apply the global config and invert since it is on the opposite side
    (rightLeaderConfig.apply(globalConfig)
     .inverted(True)
     )
    
    # Apply the global config and set the leader SPARK for follower mode
    (leftFollowerConfig.apply(globalConfig)
     .follow(self._leftLeader)
     )

    # Apply the global config and set the leader SPARK for follower mode
    (rightFollowConfig.apply(globalConfig)
     .follow(self._rightLeader))

    # Apply the configuration to the SPARKs.
    # 
    # kResetSafeParameters is used to get the SPARK MAX to a known state. This
    # is useful in case the SPARK MAX is replaced.
    # 
    # kPersistParameters is used to ensure the configuration is not lost when
    # the SPARK MAX loses power. This is useful for power cycles that may occur
    # mid-operation.
    self._leftLeader.configure(
      globalConfig,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters)
    self._leftFollower.configure(
      leftFollowerConfig,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters)
    self._rightLeader.configure(
      rightLeaderConfig,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters)
    self._rightFollower.configure(
      rightFollowConfig,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters)

    # Initialize joystick
    self.joystick = XboxController(0)

  def robotPeriodic(self):
    # Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("Left Out", self._leftLeader.getAppliedOutput())
    SmartDashboard.putNumber("Right Out", self._rightLeader.getAppliedOutput())

  def teleopPeriodic(self):
    # Get forward and rotation values from the joystick. Invert the joystick's
    # Y value because its forward direction is negative.
    forward = applyDeadzone(-self.joystick.getLeftY())
    rotation = applyDeadzone(self.joystick.getRightX())

    # Apply values to left and right side. We will only need to set the leaders
    # since the other motors are in follower mode.
    self._leftLeader.set(forward + rotation)
    self._rightLeader.set(forward - rotation)
