package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ControlPanelSubsystem;



public final class RunColorMotor extends CommandBase {
   private final ControlPanelSubsystem m_subsystem;

   public void initialize() {
   }

   public void execute() {
      this.m_subsystem.driveColorMotor(0.35D);
   }

   public void end(boolean interrupted) {
      this.m_subsystem.driveColorMotor(0.0D);
   }

   public boolean isFinished() {
      return false;
   }

   public final ControlPanelSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public RunColorMotor(ControlPanelSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
