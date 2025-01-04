#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from wpimath.trajectory import TrapezoidProfile


class Constants:
  """
  The Constants class provides a convenient place for teams to hold robot-wide
  numerical or boolean
  constants. This class should not be used for any other purpose. All constants
  should be declared
  globally (i.e. public static). Do not put anything functional in this class.
  """

class NeoMotorConstants:
  kFreeSpeedRpm: float = 5676

class DriveConstants:
  # Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather the allowed maximum speeds
  kMaxSpeedMetersPerSecond: float = 4.8
  kMaxAngularSpeed: float = 2.0 * math.pi

  # Chassis configuration
  # Distance between centers of right and left wheels on robot
  kTrackWidth: float = units.inchesToMeters(26.5)

  # Distance between front and back wheels on robot
  kWheelBase: float = units.inchesToMeters(26.5)

  kDriveKinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(
    Translation2d(kWheelBase / 2, kTrackWidth / 2),
    Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
  )

  # Angular offsets of the modules relative to the chassis in radians
  kFrontLeftChassisAngularOffset = -math.pi / 2
  kFrontRightChassisAngularOffset = 0
  kRearLeftChassisAngularOffset = math.pi
  kRearRightChassisAngularOffset = math.pi / 2

  # SPARK MAX CAN IDs
  kFrontLeftDrivingCanId: int = 11
  kRearLeftDrivingCanId: int = 13
  kFrontRightDrivingCanId: int = 15
  kRearRightDrivingCanId: int = 17

  kFrontLeftTurningCanId: int = 10
  kRearLeftTurningCanId: int = 12
  kFrontRightTurningCanId: int = 14
  kRearRightTurningCanId: int = 16

  kGyroReversed: bool = False

class ModuleConstants:
  # The MAXSwerve module can be configured with one of three pinion gears: 12T,
  # 13T, or 14T. This changes the drive speed of the module (a pinion gear with
  # more teeth will result in a robot that drives faster).
  kDrivingMotorPinionTeeth: int = 14

  # Calculations required for driving motor conversion factors and feed forward
  kDrivingMotorFreeSpeedRps: float = NeoMotorConstants.kFreeSpeedRpm / 60
  kWheelDiameterMeters: float = 0.0762
  kWheelCircumferenceMeters: float = kWheelDiameterMeters * math.pi
  # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  # teeth on the bevel pinion
  kDrivingMotorReduction: float = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
  kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

  # Simulation constants
  # 5-G acceleration
  kdrivingMotorSimSlew: float = 9.8 * 5
  # Treat the turning motor as free-spinning
  kturningMotorSimSpeed: float = 31.0
  kturningMotorSimD: float = 0.0

class OIConstants:
  kDriverControllerPort: int = 0
  kDriveDeadband: float = 0.05

class AutoConstants:
  kMaxSpeedMetersPerSecond: float = 3
  kMaxAccelerationMetersPerSecondSquared: float = 3
  kMaxAngularSpeedRadiansPerSecond: float = math.pi
  kMaxAngularSpeedRadiansPerSecondSquared: float = math.pi
  kPXController: float = 1
  kPYController: float = 1
  kPThetaController: float = 1

  # Constraint for the motion profiled robot angle controller
  kThetaControllerConstraints = TrapezoidProfile.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared)
