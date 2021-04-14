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



public class r2goBrrrrr extends SequentialCommandGroup {

    HyperdriveSubsystem hyperdrive;
    Trajectory trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.Hyperdrive.ksVolts, Constants.Hyperdrive.kvVoltsSecondsPerMeter, Constants.Hyperdrive.kaVoltSecondsSquarePerMeter),
      Constants.Hyperdrive.kDriveKinematics, 10);
    TrajectoryConfig config =
    new TrajectoryConfig(Constants.Hyperdrive.kMaxSpeedMetersPerSecond, Constants.Hyperdrive.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Hyperdrive.kDriveKinematics).addConstraint(autoVoltageConstraint);

    RamseteCommand ramseteCommand;
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }

  public r2goBrrrrr(HyperdriveSubsystem hyperdrive, Trajectory trajectory) {
    this.hyperdrive = hyperdrive;

    this.ramseteCommand = new RamseteCommand(
      trajectory, 
      hyperdrive::getPose, 
      new RamseteController(Constants.Hyperdrive.kRamseteB, Constants.Hyperdrive.kRamseteZeta) ,
      Constants.Hyperdrive.kDriveKinematics,
      hyperdrive::setWheelVelocity, 
      hyperdrive
    );
    this.hyperdrive.resetOdometry(trajectory.getInitialPose());
    this.trajectory = trajectory;

    addCommands(
      ramseteCommand.andThen(() -> hyperdrive.tankDriveVolts(0.0, 0.0))
    );
    System.out.println("Starting Automomous Path!");
  }
}
