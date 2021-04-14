// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.HyperdriveSubsystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.*;



public class r2goBounce extends SequentialCommandGroup {

    HyperdriveSubsystem hyperdrive;
    Trajectory trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.Hyperdrive.ksVolts, Constants.Hyperdrive.kvVoltsSecondsPerMeter, Constants.Hyperdrive.kaVoltSecondsSquarePerMeter),
      Constants.Hyperdrive.kDriveKinematics, 10);
    TrajectoryConfig config =
    new TrajectoryConfig(Constants.Hyperdrive.kMaxSpeedMetersPerSecond, Constants.Hyperdrive.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Hyperdrive.kDriveKinematics).addConstraint(autoVoltageConstraint);

    RamseteCommand ramseteCommand1;
    RamseteCommand ramseteCommand2;
    RamseteCommand ramseteCommand3;
    RamseteCommand ramseteCommand4;
  public boolean isFinished() {
    return ramseteCommand4.isFinished();
  }

  public r2goBounce(HyperdriveSubsystem hyperdrive, Trajectory trajectory1, Trajectory trajectory2, Trajectory trajectory3, Trajectory trajectory4) {
    this.hyperdrive = hyperdrive;

    this.ramseteCommand1 = new RamseteCommand(
      trajectory1, 
      hyperdrive::getPose, 
      new RamseteController(Constants.Hyperdrive.kRamseteB, Constants.Hyperdrive.kRamseteZeta) ,
      Constants.Hyperdrive.kDriveKinematics,
      hyperdrive::setWheelVelocity, 
      hyperdrive
    );
    this.ramseteCommand2 = new RamseteCommand(
      trajectory2, 
      hyperdrive::getPose, 
      new RamseteController(Constants.Hyperdrive.kRamseteB, Constants.Hyperdrive.kRamseteZeta) ,
      Constants.Hyperdrive.kDriveKinematics,
      hyperdrive::setReverseWheelVelocity, 
      hyperdrive
    );
    this.ramseteCommand3 = new RamseteCommand(
      trajectory3, 
      hyperdrive::getPose, 
      new RamseteController(Constants.Hyperdrive.kRamseteB, Constants.Hyperdrive.kRamseteZeta) ,
      Constants.Hyperdrive.kDriveKinematics,
      hyperdrive::setWheelVelocity, 
      hyperdrive
    );
    this.ramseteCommand4 = new RamseteCommand(
      trajectory4, 
      hyperdrive::getPose, 
      new RamseteController(Constants.Hyperdrive.kRamseteB, Constants.Hyperdrive.kRamseteZeta) ,
      Constants.Hyperdrive.kDriveKinematics,
      hyperdrive::setReverseWheelVelocity, 
      hyperdrive
    );
    this.trajectory = trajectory1;
    this.hyperdrive.resetOdometry(trajectory.getInitialPose());
    System.out.println("Created all ramsete commands for bounce");
    addCommands(
      ramseteCommand1,
      new r2SetOdometry(hyperdrive, trajectory2),
      ramseteCommand2,
      new r2SetOdometry(hyperdrive, trajectory3),
      ramseteCommand3,
      new r2SetOdometry(hyperdrive, trajectory4),
      ramseteCommand4.andThen( () -> hyperdrive.tankDriveVolts(0.0, 0.0))
    );
    System.out.println("Starting Automomous Bounces!");
  }
}
