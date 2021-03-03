package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public final class ControlPanelSubsystem extends SubsystemBase {
   private final WPI_VictorSPX colorMotor = new WPI_VictorSPX(11);

   public void periodic() {
   }

   public final void driveColorMotor(double power) {
      this.colorMotor.set(power);
   }
}
