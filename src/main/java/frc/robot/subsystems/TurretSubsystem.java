package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public final class TurretSubsystem extends SubsystemBase {
   private final WPI_VictorSPX motor = new WPI_VictorSPX(4);

   public final WPI_VictorSPX getMotor() {
      return this.motor;
   }

   public void periodic() {
   }
}
