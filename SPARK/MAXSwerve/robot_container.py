#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpimath
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drive_subsystem import DriveSubsystem
from commands2 import Command, RunCommand, SwerveControllerCommand
from commands2.button import CommandXboxController


class RobotContainer:
  """
  This class is where the bulk of the robot should be declared. Since Command-based is a
  "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
  periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
  subsystems, commands, and button mappings) should be declared here.

  """

  def __init__(self):
    # The robot's subsystems
    self._robotDrive = DriveSubsystem()

    # The driver's controller
    self._driverController = CommandXboxController(OIConstants.kDriverControllerPort)

    # Configure the button bindings
    self.configureButtonBindings()
    
    self._robotDrive.setDefaultCommand(
      # The left stick controls translation of the robot.
      # Turning is controlled by the X axis of the right stick.
      RunCommand(
        lambda: self._robotDrive.drive(
          -wpimath.applyDeadband(self._driverController.getLeftY(), OIConstants.kDriveDeadband),
          -wpimath.applyDeadband(self._driverController.getLeftX(), OIConstants.kDriveDeadband),
          -wpimath.applyDeadband(self._driverController.getRightX(), OIConstants.kDriveDeadband),
          True
        ),
        self._robotDrive
      )
    )

  def configureButtonBindings(self):
    """
    Use this method to define your button->command mappings. Buttons can be created via the button
    factories on commands2.button.CommandGenericHID or one of its
    subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
    """
    self._driverController.rightBumper().whileTrue(
      commands2.InstantCommand(
        (lambda: self._robotDrive.setX()), self._robotDrive
      )
    )

  def getAutonomousCommand(self) -> commands2.Command:
    """
    Use this to pass the autonomous command to the main :class:`.Robot` class.

    :returns: the command to run in autonomous
    """
    config = TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(DriveConstants.kDriveKinematics)

    # An example trajectory to follow. All units in meters.
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      # Start at the origin facing the +X direction
      Pose2d(0, 0, Rotation2d()),
      # Pass through these two interior waypoints, making an 's' curve path
      [
        Translation2d(1, 1),
        Translation2d(2, -1)
      ],
      # End 3 meters straight ahead of where we started, facing forward
      Pose2d(3, 0, Rotation2d(0)),
      config)

    thetaController = ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints)
    thetaController.enableContinuousInput(-math.pi, math.pi)

    swerveControllerCommand = SwerveControllerCommand(
      exampleTrajectory,
      lambda: self._robotDrive.getPose(),
      DriveConstants.kDriveKinematics,

      # Position controllers
      PIDController(AutoConstants.kPXController, 0, 0),
      PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      lambda states: self._robotDrive.setModuleStates(states),
      self._robotDrive
    )

    # Reset odometry to the starting pose of the trajectory
    self._robotDrive.resetOdometry(exampleTrajectory.initialPose())

    # Run path following command, then stop at the end.
    return swerveControllerCommand.finallyDo(lambda: self._robotDrive.drive(0, 0, 0, False))
