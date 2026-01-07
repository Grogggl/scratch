#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpimath
import wpilib

from rev import SparkMax
from commands2 import cmd, RunCommand # ADD RunCommand for example 1
from commands2.button import CommandXboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intake import Intake  #  Added with intake subsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        self.intake = Intake(leaderCanID=10, leaderInverted=True, followerCanID=11, followerInverted=False)
        # ^^ you can also specify followerCanID= and followerInverted= if the intake has a follower motor

        # The driver's controller
        # !Must match commands below, i.e CommandXboxController utilizes the commands2.button.Trigger
        self.driverController = CommandXboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # example 1: hold wheels in "swerve X brake" position while "X" button pressed
        # simple code entirely within binding
        brakeCommand = RunCommand(self.robotDrive.setX, self.robotDrive)
        self.driverController.x().whileTrue(brakeCommand)

        # example 2: Run sparkMax motor controller
        # for competition more contstants to constants.py
        self.shooterBottom = SparkMax(10, SparkMax.MotorType.kBrushed)
        self.shooterMiddle = SparkMax(11, SparkMax.MotorType.kBrushed)
        self.shooterFront = SparkMax(13, SparkMax.MotorType.kBrushless)
        self.shooterBack = SparkMax(12, SparkMax.MotorType.kBrushless)
        self.shooterFront.setInverted(True)
        self.shooter = wpilib.MotorControllerGroup(self.shooterBack, self.shooterFront, self.shooterMiddle, self.shooterBottom)

        self.driverController.y().onTrue(cmd.runOnce(lambda:self.shooter.set(0.2)))
        self.driverController.y().onFalse(cmd.runOnce(lambda:self.shooter.set(0.0)))
        self.driverController.b().onTrue(cmd.runOnce(lambda:self.shooter.set(-0.2)))
        self.driverController.b().onFalse(cmd.runOnce(lambda:self.shooter.set(0.0)))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(.3, .3), Translation2d(.6, -.3)],
            # End .75 meters straight ahead of where we started, facing forward
            Pose2d(1, 0, Rotation2d(0)),
            config,
        )

        # Constraint for the motion profiled robot angle controller
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
        )

        kPXController = PIDController(1.0, 0.0, 0.0)
        kPYController = PIDController(1.0, 0.0, 0.0)
        kPThetaController = ProfiledPIDControllerRadians(
            1.0, 0.0, 0.0, kThetaControllerConstraints
        )
        kPThetaController.enableContinuousInput(-math.pi, math.pi)

        kPIDController = HolonomicDriveController(
            kPXController, kPYController, kPThetaController
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            kPIDController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )
