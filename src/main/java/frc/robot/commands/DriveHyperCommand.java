package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HyperdriveSubsystem;



public final class DriveHyperCommand extends CommandBase {
   private final HyperdriveSubsystem hyperdriveSubsystem;
   private final Joystick leftJoystick;
   private final Joystick rightJoystick;

   public void initialize() {
   }

   public void execute() {
      // System.out.println(hyperdriveSubsystem.getPose());
      // System.out.println("Left: " + this.hyperdriveSubsystem.leftEncoderPos() + "  Right: " + this.hyperdriveSubsystem.rightEncoderPos());
      // System.out.println(hyperdriveSubsystem.getTranslation());

      double leftPower = 0.0D;
      double rightPower = 0.0D;
      double leftJoystickVal = this.leftJoystick.getY();
      double rightJoystickVal = this.rightJoystick.getY();
      if (Math.abs(leftJoystickVal) > 0.15D) {
         leftPower = leftJoystickVal;
      }

      if (Math.abs(rightJoystickVal) > 0.15D) {
         rightPower = rightJoystickVal;
      }

      if (this.hyperdriveSubsystem.getRobotDirectionInverted()) {
         this.hyperdriveSubsystem.tankDrive(-rightPower, -leftPower);
      } else {
         this.hyperdriveSubsystem.tankDrive(leftPower, rightPower);
      }

   }

   public void end(boolean interrupted) {
      this.hyperdriveSubsystem.tankDrive(0.0D, 0.0D);
   }

   public boolean isFinished() {
      return false;
   }

   public DriveHyperCommand(HyperdriveSubsystem hyperdriveSubsystem, Joystick leftJoystick, Joystick rightJoystick) {
      super();
     
      this.hyperdriveSubsystem = hyperdriveSubsystem;
      this.leftJoystick = leftJoystick;
      this.rightJoystick = rightJoystick;
      this.addRequirements(new Subsystem[]{(Subsystem)this.hyperdriveSubsystem});
   }
}
