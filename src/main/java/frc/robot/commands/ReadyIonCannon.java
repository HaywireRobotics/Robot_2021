package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IonCannonSubsystem;



public final class ReadyIonCannon extends CommandBase {
   private final IonCannonSubsystem ionCannon;

   public void initialize() {
      this.ionCannon.setSetpoints(100000.0, 100000.0);
   }

   public boolean isFinished() {
      return true;
   }

   public ReadyIonCannon(IonCannonSubsystem ionCannon) {
      super();
      this.ionCannon = ionCannon;
   }
}
