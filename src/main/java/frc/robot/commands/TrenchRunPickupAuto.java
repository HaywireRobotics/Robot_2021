package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DockingBaySubsystem;
import frc.robot.subsystems.HyperdriveSubsystem;
import frc.robot.subsystems.IonCannonSubsystem;
import frc.robot.subsystems.TurboLiftSubsystem;
import frc.robot.subsystems.TurretSubsystem;



public final class TrenchRunPickupAuto extends SequentialCommandGroup {
   public TrenchRunPickupAuto(IonCannonSubsystem ionCannonSubsystem, TurretSubsystem turret, TurboLiftSubsystem turboLift, DockingBaySubsystem dockingBay, HyperdriveSubsystem driveTrain) {
      super(new Command[0]);
      this.addCommands(new Command[]{(Command)(new TurretSeekAutonomous(turret)), 
         (Command)(new LaunchIonCannonForTime(160000.0, 160000.0, 7.0, ionCannonSubsystem, turboLift)), 
         (Command)(new ParallelCommandGroup(new Command[]{
            (Command)(new RunDockingBayForTime(dockingBay, -0.7D, 15.0)), 
            (Command)(new DriveForTime(driveTrain, -0.25D, 2.25D))}))});
   }
}
