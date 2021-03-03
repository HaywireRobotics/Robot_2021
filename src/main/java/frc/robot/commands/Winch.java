package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;


public final class Winch extends CommandBase {

   private final ClimbSubsystem m_subsystem;

   public void initialize() {
   }

   public void execute() {
      this.m_subsystem.driveWinchMotor(-1.0D);
   }

   public void end(boolean interrupted) {
      this.m_subsystem.driveWinchMotor(0.0D);
   }

   public boolean isFinished() {
      return false;
   }


   public final ClimbSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public Winch(ClimbSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
