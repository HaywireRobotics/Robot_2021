package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public final class TurboLiftSubsystem extends SubsystemBase {
   private final WPI_VictorSPX backMotor = new WPI_VictorSPX(2);
   private final WPI_VictorSPX frontMotor = new WPI_VictorSPX(40);
   private final WPI_VictorSPX agitator = new WPI_VictorSPX(1);
   // private final int maxPulseCount = 100;
   // private int pulseCount;
   private double agitatorSpeed = -0.15D;

   public final WPI_VictorSPX getBackMotor() {
      return this.backMotor;
   }

   public final WPI_VictorSPX getFrontMotor() {
      return this.frontMotor;
   }

   public void periodic() {
   }

   public final void runSystem(double elevatorSpeed) {
      // boolean var3 = false;
      if (Math.abs(elevatorSpeed) > 0.1D) {
         this.frontMotor.set(-elevatorSpeed);
         this.backMotor.set(elevatorSpeed);
         this.agitator.set(this.agitatorSpeed);
      } else {
         this.frontMotor.set(0.0D);
         this.backMotor.set(0.0D);
         this.agitator.set(0.0D);
      }

   }
}
