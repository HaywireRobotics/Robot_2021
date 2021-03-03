package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TurretSubsystem;



public final class TurretManualDrive extends CommandBase {
   private final TurretSubsystem m_subsystem;
   private final Joystick joystick;

   public void initialize() {
   }

   public void execute() {
      this.m_subsystem.getMotor().set(this.joystick.getX() / (double)3);
   }

   public void end(boolean interrupted) {
   }

   public boolean isFinished() {
      return false;
   }

   public final TurretSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public TurretManualDrive(TurretSubsystem m_subsystem, Joystick joystick) {
      super();
      this.m_subsystem = m_subsystem;
      this.joystick = joystick;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
