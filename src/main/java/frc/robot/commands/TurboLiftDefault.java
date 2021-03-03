package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TurboLiftSubsystem;

public final class TurboLiftDefault extends CommandBase {
   private double joystickPower;
   private final TurboLiftSubsystem m_subsystem;
   private final Joystick joystick;

   public void initialize() {
   }

   public void execute() {
      this.joystickPower = this.joystick.getY();
      this.m_subsystem.runSystem(this.joystickPower);
   }

   public void end(boolean interrupted) {
      this.m_subsystem.runSystem(0.0D);
   }

   public boolean isFinished() {
      return false;
   }

   public final TurboLiftSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public TurboLiftDefault(TurboLiftSubsystem m_subsystem, Joystick joystick) {
      super();
      this.m_subsystem = m_subsystem;
      this.joystick = joystick;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
