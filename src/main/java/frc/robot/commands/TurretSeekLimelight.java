package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TurretSubsystem;



public final class TurretSeekLimelight extends CommandBase {
   private final PIDController pidController;
   private final NetworkTableInstance nt;
   private final NetworkTable table;
   private final NetworkTableEntry tx;
   private final NetworkTableEntry light;
   private int waitCounter;
   private final TurretSubsystem m_subsystem;

   public void initialize() {
      this.pidController.reset();
      this.light.setNumber((Number)3);
      this.waitCounter = 20;
   }

   public void execute() {
      if (this.waitCounter > 0) {
         --this.waitCounter;
      } else {
         this.useOutput(this.pidController.calculate(this.generateMeasurement(), this.generateSetpoint()));
      }

   }

   public void end(boolean interrupted) {
      this.useOutput(0.0D);
      this.light.setNumber((Number)0);
   }

   public boolean isFinished() {
      return false;
   }

   private final void useOutput(double output) {
      this.m_subsystem.getMotor().set(-output);
   }

   private final double generateMeasurement() {
      return this.tx.getDouble(0.0D);
   }

   private final double generateSetpoint() {
      return 0.0D;
   }

   public final TurretSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public TurretSeekLimelight(TurretSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
      NetworkTableInstance var10001 = NetworkTableInstance.getDefault();
      this.nt = var10001;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
      this.pidController = new PIDController(0.027D, 0.010D, 0.0027D);
      NetworkTable var2 = this.nt.getTable("limelight");
      this.table = var2;
      NetworkTableEntry var3 = this.table.getEntry("tx");
      this.tx = var3;
      var3 = this.table.getEntry("ledMode");
      this.light = var3;
      this.nt.startClientTeam(1569);
   }
}
