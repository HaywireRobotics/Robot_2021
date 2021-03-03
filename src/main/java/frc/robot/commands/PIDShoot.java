package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IonCannonSubsystem;


public final class PIDShoot extends CommandBase {
   private final double bottomTargetRate;
   private final double topTargetRate;
   private final IonCannonSubsystem m_subsystem;

   public void initialize() {
      this.m_subsystem.setSetpoints(this.bottomTargetRate, this.topTargetRate);
      this.m_subsystem.resetPID();
   }

   public void execute() {
      this.m_subsystem.runPID();
   }

   public void end(boolean interrupted) {
      this.m_subsystem.endPID();
   }

   public boolean isFinished() {
      return false;
   }

   public final IonCannonSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public PIDShoot(double bottomTargetRate, double topTargetRate, IonCannonSubsystem m_subsystem) {
      super();
      this.bottomTargetRate = bottomTargetRate;
      this.topTargetRate = topTargetRate;
      this.m_subsystem = m_subsystem;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
