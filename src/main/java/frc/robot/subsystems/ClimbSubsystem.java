package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public final class ClimbSubsystem extends SubsystemBase {
   private final WPI_VictorSPX hookMotor = new WPI_VictorSPX(12);
   private final WPI_VictorSPX winchMotor = new WPI_VictorSPX(5);

   public void periodic() {
   }

   public final void driveHookMotor(double power) {
      this.hookMotor.set(power);
   }

   public final void driveWinchMotor(double power) {
      this.winchMotor.set(power);
   }
}
