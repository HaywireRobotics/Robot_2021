package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HyperdriveSubsystem;



public final class DriveForTime extends CommandBase {
   private final Timer timer;
   private final HyperdriveSubsystem m_subsystem;
   private final double speed;
   private final double time;

   public void initialize() {
      this.timer.reset();
      this.timer.start();
   }

   public void execute() {
      this.m_subsystem.tankDrive(this.speed, this.speed);
   }

   public void end(boolean interrupted) {
      this.m_subsystem.tankDrive(0.0D, 0.0D);
   }

   public boolean isFinished() {
      return this.timer.hasPeriodPassed(this.time);
   }

   public final HyperdriveSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public final double getTime() {
      return this.time;
   }

   public DriveForTime(HyperdriveSubsystem m_subsystem, double speed, double time) {
      super();
      
      this.m_subsystem = m_subsystem;
      this.speed = speed;
      this.time = time;
      this.timer = new Timer();
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
