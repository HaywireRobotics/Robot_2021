package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DockingBaySubsystem;
import frc.robot.subsystems.IonCannonSubsystem;
import frc.robot.subsystems.TurboLiftSubsystem;


public final class LaunchIonCannon extends CommandBase {
   private double initialZAxisValue;
   private final double multiplier;
   private final double topTargetRate;
   private final double bottomTargetRate;
   private final IonCannonSubsystem ionCannonSubsystem;
   private final TurboLiftSubsystem turboLift;
   private final DockingBaySubsystem dockingBay;
   private final Joystick joystick;
   private boolean CannonCharged;
   private int intakeSpin;


   public void initialize() {
      this.ionCannonSubsystem.setSetpoints(this.topTargetRate, this.bottomTargetRate);
      this.ionCannonSubsystem.resetPID();
      this.initialZAxisValue = this.joystick.getZ();
      CannonCharged = false;
      intakeSpin = 0;
   }

   public void execute() {
      this.ionCannonSubsystem.runPID();
      if (this.ionCannonSubsystem.isReady() || CannonCharged) {
         CannonCharged = true;
         this.turboLift.runSystem(-0.6D);
         if (intakeSpin < 20) {
            intakeSpin++;
            dockingBay.getMotor().set(-0.35);
         } else {
            dockingBay.getMotor().set(0);
         }
      }

      this.ionCannonSubsystem.setSetpoints(this.topTargetRate + (this.joystick.getZ() - this.initialZAxisValue) * this.multiplier, this.bottomTargetRate+ (this.joystick.getZ() - this.initialZAxisValue) * this.multiplier);
   }

   public void end(boolean interrupted) {
      this.ionCannonSubsystem.endPID();
   }

   public boolean isFinished() {
      return false;
   }

   public LaunchIonCannon(Double topTargetRate, Double bottomTargetRate, IonCannonSubsystem ionCannonSubsystem, TurboLiftSubsystem turboLift, Joystick joystick, DockingBaySubsystem dockingBay) {
      super();
      
      this.topTargetRate = topTargetRate;
      this.bottomTargetRate = bottomTargetRate;
      this.ionCannonSubsystem = ionCannonSubsystem;
      this.dockingBay = dockingBay;
      this.turboLift = turboLift;
      this.joystick = joystick;
      this.multiplier = 10000.0D;
      this.addRequirements(new Subsystem[]{(Subsystem)this.ionCannonSubsystem});
   }
}
