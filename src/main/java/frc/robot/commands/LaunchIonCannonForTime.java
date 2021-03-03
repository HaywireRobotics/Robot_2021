package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IonCannonSubsystem;
import frc.robot.subsystems.TurboLiftSubsystem;



public final class LaunchIonCannonForTime extends CommandBase {
   private final Timer timer;
   private final double topTargetRate;
   private final double bottomTargetRate;
   private final double shootTime;
   private final IonCannonSubsystem ionCannonSubsystem;
   private final TurboLiftSubsystem turboLift;

   public void initialize() {
      this.timer.reset();
      this.timer.start();
      this.ionCannonSubsystem.setSetpoints(this.topTargetRate, this.bottomTargetRate);
      this.ionCannonSubsystem.resetPID();
   }

   public void execute() {
      this.ionCannonSubsystem.runPID();
      if (this.ionCannonSubsystem.isReady()) {
         this.turboLift.runSystem(-0.6D);
      }

   }

   public void end(boolean interrupted) {
      this.ionCannonSubsystem.endPID();
      this.turboLift.runSystem(0.0D);
   }

   public boolean isFinished() {
      return this.timer.hasPeriodPassed(this.shootTime);
   }

   public LaunchIonCannonForTime(Double topTargetRate, Double bottomTargetRate, Double shootTime, IonCannonSubsystem ionCannonSubsystem, TurboLiftSubsystem turboLift) {
      super();

      this.topTargetRate = topTargetRate;
      this.bottomTargetRate = bottomTargetRate;
      this.shootTime = shootTime;
      this.ionCannonSubsystem = ionCannonSubsystem;
      this.turboLift = turboLift;
      this.timer = new Timer();
      this.addRequirements(new Subsystem[]{(Subsystem)this.ionCannonSubsystem, (Subsystem)this.turboLift});
   }
}
