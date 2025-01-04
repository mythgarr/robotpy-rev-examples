#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from rev import SparkMax
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Odometry, SwerveDrive4Kinematics
from commands2 import Subsystem
from .max_swerve_module import MAXSwerveModule
from constants import DriveConstants
from wpilib import ADIS16470_IMU
from ntcore import NetworkTableInstance

IMUAxis = ADIS16470_IMU.IMUAxis


class DriveSubsystem(Subsystem):
  def __init__(self):
    # Create MAXSwerveModules
    self._frontLeft = MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset
    )
    self._frontRight = MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset
    )
    self._rearLeft = MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset
    )
    self._rearRight = MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset
    )

    # The gyro sensor
    self._gyro = ADIS16470_IMU()

    networkTable = NetworkTableInstance.getDefault()
    
    self._desiredStatePublisher = networkTable.getStructArrayTopic("Swerve/Modules/DesiredStates", SwerveModuleState).publish()
    self._statePublisher = networkTable.getStructArrayTopic("Swerve/Modules/States", SwerveModuleState).publish()
    self._publisher = networkTable.getStructTopic("Swerve/Pose", Pose2d).publish()

    # Odometry class for tracking robot pose
    self._odometry = SwerveDrive4Odometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      )
    )

  def periodic(self) -> None:
    self._odometry.update(
      Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      )
    )
    self.updateTelemetry()

  def updateTelemetry(self) -> None:
    self._publisher.set(self.getPose())

    self._statePublisher.set([
      self._frontLeft.getState(),
      self._frontRight.getState(),
      self._rearLeft.getState(),
      self._rearRight.getState()]
    )

    self._desiredStatePublisher.set([
      self._frontLeft.getDesiredState(),
      self._frontRight.getDesiredState(),
      self._rearLeft.getDesiredState(),
      self._rearRight.getDesiredState()
    ])

  def getPose(self) -> Pose2d:
    """
    Returns the currently-estimated pose of the robot.
    :return: The pose.
    """
    return self._odometry.getPose()

  def resetOdometry(self, pose: Pose2d) -> None:
    """
    Resets the odometry to the specified pose.
    :param pose: The pose to which to set the odometry.
    """
    self._odometry.resetPosition(
      Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      ),
      pose)

  def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool) -> None:
    """
    Method to drive the robot using joystick info.
    :param xSpeed: Speed of the robot in the x direction (forward).
    :param ySpeed: Speed of the robot in the y direction (sideways).
    :param rot: Angular rate of the robot.
    :param fieldRelative: Whether the provided x and y speeds are relative to the field.
    """
    # Convert the commanded speeds into the correct units for the drivetrain
    xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
    ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
    rotDelivered = rot * DriveConstants.kMaxAngularSpeed

    swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                            Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ))) if fieldRelative
      else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

    SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)

    self._frontLeft.setDesiredState(swerveModuleStates[0])
    self._frontRight.setDesiredState(swerveModuleStates[1])
    self._rearLeft.setDesiredState(swerveModuleStates[2])
    self._rearRight.setDesiredState(swerveModuleStates[3])

  def setX(self) -> None:
    """
    Sets the wheels into an X formation to prevent movement.
    """
    self._frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
    self._frontRight.setDesiredState((0, Rotation2d.fromDegrees(-45)))
    self._rearLeft.setDesiredState((0, Rotation2d.fromDegrees(-45)))
    self._rearRight.setDesiredState((0, Rotation2d.fromDegrees(45)))

  def setModuleStates(self, desiredStates: tuple[SwerveModuleState, ...]) -> None:
    """
    Sets the swerve ModuleStates.
    :param desiredStates: The desired SwerveModule states.
    """
    SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)
    self._frontLeft.setDesiredState(desiredStates[0])
    self._frontRight.setDesiredState(desiredStates[1])
    self._rearLeft.setDesiredState(desiredStates[2])
    self._rearRight.setDesiredState(desiredStates[3])

  def resetEncoders(self) -> None:
    """
    Resets the drive encoders to currently read a position of 0.
    """
    self._frontLeft.resetEncoders()
    self._rearLeft.resetEncoders()
    self._frontRight.resetEncoders()
    self._rearRight.resetEncoders()

  def zeroHeading(self) -> None:
    """
    Zeroes the heading of the robot.
    """
    self._gyro.reset()

  def getHeading(self) -> float:
    """
    Returns the heading of the robot.
    :return: The robot's heading in degrees, from -180 to 180
    """
    return Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)).getDegrees()

  def getTurnRate(self) -> float:
    """
    Returns the turn rate of the robot.
    :return: The turn rate of the robot, in degrees per second
    """
    multiplier = -1.0 if DriveConstants.kGyroReversed else 1.0
    return self._gyro.getRate(IMUAxis.kZ) * multiplier
