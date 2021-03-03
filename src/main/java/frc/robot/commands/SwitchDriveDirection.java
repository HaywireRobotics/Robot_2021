package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HyperdriveSubsystem;



public final class SwitchDriveDirection extends CommandBase {
   private final HyperdriveSubsystem m_subsystem;

   public void initialize() {
   }

   public void execute() {
      HyperdriveSubsystem var10000 = this.m_subsystem;
      boolean var1 = this.m_subsystem.getRobotDirectionInverted();
      boolean var10001;
      if (!var1) {
         var10001 = true;
      } else {
   
         var10001 = false;
      }

      var10000.setRobotDirectionInverted(var10001);
   }

   public void end(boolean interrupted) {
   }

   public boolean isFinished() {
      return true;
   }

   public final HyperdriveSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public SwitchDriveDirection(HyperdriveSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
   }
}
