package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IonCannonSubsystem;
import frc.robot.subsystems.TurboLiftSubsystem;



public final class TestShooterNT extends CommandBase {
   private final NetworkTableInstance nt;
   private final NetworkTable table;
   private final NetworkTableEntry topShooterSpeedEntry;
   private final NetworkTableEntry bottomShooterSpeedEntry;
   private double topTargetRate;
   private double bottomTargetRate;
   private final IonCannonSubsystem ionCannonSubsystem;
   private final TurboLiftSubsystem turboLift;

   public void initialize() {
      this.topTargetRate = this.topShooterSpeedEntry.getDouble(this.topTargetRate);
      this.bottomTargetRate = this.bottomShooterSpeedEntry.getDouble(this.bottomTargetRate);
      String var1 = "TOP Setpoint: " + this.topTargetRate;
      // boolean var2 = false;
      System.out.println(var1);
      var1 = "BOTTOM Setpoint: " + this.bottomTargetRate;
      // var2 = false;
      System.out.println(var1);
      this.ionCannonSubsystem.setSetpoints(this.bottomTargetRate, this.topTargetRate);
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
   }

   public boolean isFinished() {
      return false;
   }

   public TestShooterNT(IonCannonSubsystem ionCannonSubsystem, TurboLiftSubsystem turboLift) {
      super();
      this.ionCannonSubsystem = ionCannonSubsystem;
      this.turboLift = turboLift;
      NetworkTableInstance var10001 = NetworkTableInstance.getDefault();
      this.nt = var10001;
      this.addRequirements(new Subsystem[]{(Subsystem)this.ionCannonSubsystem});
      NetworkTable var3 = this.nt.getTable("SmartDashboard");
      this.table = var3;
      NetworkTableEntry var4 = this.table.getEntry("topShooterSpeed");
      this.topShooterSpeedEntry = var4;
      var4 = this.table.getEntry("bottomShooterSpeed");
      this.bottomShooterSpeedEntry = var4;
      this.nt.startClientTeam(1569);
   }
}
