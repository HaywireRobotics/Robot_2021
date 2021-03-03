package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DockingBaySubsystem;


public final class RunDockingBay extends CommandBase {
   private final DockingBaySubsystem m_subsystem;
   private final PowerDistributionPanel pdp;
   private final double speed;

   public void initialize() {
   }

   public void execute() {
      this.m_subsystem.getMotor().set(this.speed);
   }

   public void end(boolean interrupted) {
      this.m_subsystem.getMotor().set(0.0D);
   }

   public boolean isFinished() {
      return false;
   }

   public final DockingBaySubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public final PowerDistributionPanel getPdp() {
      return this.pdp;
   }

   public RunDockingBay(DockingBaySubsystem m_subsystem, PowerDistributionPanel pdp, double speed) {
      super();
      this.m_subsystem = m_subsystem;
      this.pdp = pdp;
      this.speed = speed;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
