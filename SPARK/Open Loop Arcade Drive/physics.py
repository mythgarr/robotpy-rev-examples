
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpimath import units
from wpimath.system.plant import DCMotor
from wpilib import RobotBase, RobotController
from wpilib.simulation import DifferentialDrivetrainSim

from rev import SparkMaxSim, SparkSim

import typing
from robot import MyRobot

kDeltaTime = 1.0 / 50

class PhysicsEngine:
  def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
    self._robot = robot
    self.physics_controller = physics_controller
    self._leftSim = SparkMaxSim(robot._leftLeader, DCMotor.NEO(1))
    self._rightSim = SparkMaxSim(robot._rightLeader, DCMotor.NEO(1))
    self._drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
      # Dual NEOs per-side
      DCMotor.NEO(2),
      # 10.71:1 gearing
      10.71,
      # 6-inch wheels
      units.inchesToMeters(6)
    )

  def update_sim(self, now: float, tm_diff: float) -> None:
    vBus = RobotController.getBatteryVoltage()
    if self._robot.isEnabled():
      leftOutput = self._leftSim.getAppliedOutput()
      rightOutput = self._rightSim.getAppliedOutput()
      self._drivetrainSim.setInputs(
        leftOutput * vBus,
        rightOutput * vBus
      )
      self._drivetrainSim.update(kDeltaTime)
      self._leftSim.iterate(
        self._drivetrainSim.getLeftVelocity(),
        vBus,
        kDeltaTime)
      self._rightSim.iterate(
        self._drivetrainSim.getRightVelocity(),
        vBus,
        kDeltaTime)
    else:
      self._drivetrainSim.setInputs(0, 0)
      self._drivetrainSim.update(vBus)
      self._leftSim.iterate(
        0,
        vBus,
        kDeltaTime
      )
      self._rightSim.iterate(
        0,
        vBus,
        kDeltaTime
      )
