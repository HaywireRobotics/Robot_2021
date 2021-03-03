package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public final class ColorSensorSubsystem extends SubsystemBase {
   private ColorSensorV3 colorSensor;

   public void periodic() {
   }

   public final Color getColorRGB() {
      Color var10000 = this.colorSensor.getColor();
      return var10000;
   }

   public final Number getRawIR() {
      return (Number)this.colorSensor.getIR();
   }

   public ColorSensorSubsystem() {
      Port i2cPort = Port.kOnboard;
      this.colorSensor = new ColorSensorV3(i2cPort);
   }
}
