package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TurretSubsystem;


public final class TurretSeekAutonomous extends CommandBase {
   private final PIDController pidController;
   private final NetworkTableInstance nt;
   private final NetworkTable table;
   private final NetworkTableEntry angleEntry;
   private final double marginOfError;
   private final Timer timer;

   private final TurretSubsystem m_subsystem;

   public void initialize() {
      this.pidController.reset();
      this.timer.reset();
      this.timer.start();
   }

   public void execute() {
      this.useOutput(this.pidController.calculate(this.generateMeasurement(), this.generateSetpoint()));
   }

   public void end(boolean interrupted) {
      this.useOutput(0.0D);
   }

   public boolean isFinished() {
      double angle = this.angleEntry.getDouble(0.0D);
      return angle < this.marginOfError && angle > -this.marginOfError || this.timer.hasPeriodPassed(2.0D);
   }

   private final void useOutput(double output) {
      this.m_subsystem.getMotor().set((double)1 * output);
   }

   private final double generateMeasurement() {
      return this.angleEntry.getDouble(0.0D);
   }

   private final double generateSetpoint() {
      return 0.0D;
   }


   public final TurretSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public TurretSeekAutonomous(TurretSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
      NetworkTableInstance var10001 = NetworkTableInstance.getDefault();
      this.nt = var10001;
      this.marginOfError = 1.0D;
      this.timer = new Timer();
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
      this.pidController = new PIDController(0.025D, 0.008D, 0.0025D);
      NetworkTable var2 = this.nt.getTable("datatable");
      this.table = var2;
      NetworkTableEntry var3 = this.table.getEntry("Angle");
      this.angleEntry = var3;
      this.nt.startClientTeam(1569);
   }
}
