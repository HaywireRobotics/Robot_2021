package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DockingBaySubsystem;

public final class RunDockingBayForTime extends CommandBase {
   private final Timer timer;
   private final DockingBaySubsystem m_subsystem;
   private final Number speed;
   private final Number time;

   public void initialize() {
      this.timer.reset();
      this.timer.start();
   }

   public void execute() {
      this.m_subsystem.getMotor().set(this.speed.doubleValue());
   }

   public void end(boolean interrupted) {
      this.m_subsystem.getMotor().set(0.0D);
   }

   public boolean isFinished() {
      return this.timer.hasPeriodPassed(this.time.doubleValue());
   }

   public final DockingBaySubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public final Number getTime() {
      return this.time;
   }

   public RunDockingBayForTime(DockingBaySubsystem m_subsystem, Double speed, Double time) {
       super();
      this.m_subsystem = m_subsystem;
      this.speed = speed;
      this.time = time;
      this.timer = new Timer();
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
