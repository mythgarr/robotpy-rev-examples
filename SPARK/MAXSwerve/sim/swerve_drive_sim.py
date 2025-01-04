import math
import random
import wpimath
from subsystems.max_swerve_module import MAXSwerveModule
from rev import SparkMaxSim
from wpimath.system.plant import DCMotor
from wpimath.filter import SlewRateLimiter
from constants import DriveConstants, ModuleConstants
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath import units
from subsystems.drive_subsystem import DriveSubsystem
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from wpilib.simulation import ADIS16470_IMUSim
from ntcore import NetworkTableInstance
from wpilib import RobotController


def clamp(val: float, a: float, b: float) -> float:
  return max(a, min(b, val))


def clampPose(pose: Pose2d) -> Pose2d:
  translation: Translation2d = pose.translation()
  translation = Translation2d(
    clamp(translation.X(), 0, 16.49),
    clamp(translation.Y(), 0, 8.10)
  )
  return Pose2d(translation, pose.rotation())


class SwerveDriveSim:
  class SwerveModuleSim:
    def __init__(self, module: MAXSwerveModule):
      self._driveSparkSim = SparkMaxSim(module._drivingSpark, DCMotor.NEO(1))
      self._turnSparkSim = SparkMaxSim(module._turningSpark, DCMotor.NEO(1))
      self._angularOffset = module._chassisAngularOffset

      # Calculates the velocity of the drive motor, simulating inertia and friction
      self._driveRateLimiter = SlewRateLimiter(ModuleConstants.kdrivingMotorSimSlew)

      # Calculates the velocity of the turn motor
      self._turnController = PIDController(
        ModuleConstants.kturningMotorSimSpeed,
        0,
        ModuleConstants.kturningMotorSimD
      )
      self._turnController.enableContinuousInput(-math.pi, math.pi)

      # Randomize starting rotation
      self._turnSparkSim.setPosition(random.uniform(-math.pi, math.pi))

    def simulationPeriodic(self, vbus: float, dt: float):
      # m/s
      targetVelocity = self._driveSparkSim.getSetpoint()
      driveVelocity = self._driveRateLimiter.calculate(targetVelocity)
      self._driveSparkSim.iterate(
        driveVelocity,
        vbus,
        dt
      )

      # rad
      targetAngle = self._turnSparkSim.getSetpoint()
      curAngle = wpimath.angleModulus(self._turnSparkSim.getPosition())
      self._turnController.setSetpoint(targetAngle)
      turnVelocity = self._turnController.calculate(curAngle)
      self._turnSparkSim.iterate(
        turnVelocity,
        vbus,
        dt
      )

    def getState(self) -> SwerveModuleState:
      return SwerveModuleState(
        units.meters_per_second(self._driveSparkSim.getVelocity()),
        Rotation2d(self._turnSparkSim.getPosition() - self._angularOffset)
      )

  def __init__(self, driveSubsystem: DriveSubsystem):
    self._gyroSim = ADIS16470_IMUSim(driveSubsystem._gyro)

    self._kinematics = DriveConstants.kDriveKinematics
    self._modules = (
      SwerveDriveSim.SwerveModuleSim(driveSubsystem._frontLeft),
      SwerveDriveSim.SwerveModuleSim(driveSubsystem._frontRight),
      SwerveDriveSim.SwerveModuleSim(driveSubsystem._rearLeft),
      SwerveDriveSim.SwerveModuleSim(driveSubsystem._rearRight)
    )
    self._pose = Pose2d()

    # Publish the simulated pose. This is the ACTUAL pose of the robot - the drive
    # system pose is an estimation based on encoder and vision (if applicable) data.
    self._posePublisher = NetworkTableInstance.getDefault().getStructTopic("Sim/Pose", Pose2d).publish()
    self._posePublisher.set(self._pose)

  def simulationPeriodic(self, dt: float) -> None:
    vbus = RobotController.getBatteryVoltage()
    for module in self._modules:
      module.simulationPeriodic(vbus, dt)
    moduleStates = tuple(module.getState() for module in self._modules)
    chassisSpeeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(moduleStates)
    twist = chassisSpeeds.toTwist2d(dt)

    # Constraint the pose within the Field
    self._pose = clampPose(self._pose.exp(twist))

    # Publish the simulated pose
    self._posePublisher.set(self._pose)

    # Update the gyro heading
    self._gyroSim.setGyroAngleZ(self._pose.rotation().degrees())

  def getPose(self) -> Pose2d:
    return self._pose

  def setPose(self, pose: Pose2d) -> None:
    self._pose = pose
