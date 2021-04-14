package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.io.IOException;
import java.util.List;
import java.util.ArrayList;




public final class RobotContainer {

   private final HyperdriveSubsystem hyperdriveSubsystem = new HyperdriveSubsystem();
   private final DockingBaySubsystem dockingBaySubsystem = new DockingBaySubsystem();
   private final TurboLiftSubsystem turboLiftSubsystem = new TurboLiftSubsystem();
   private final IonCannonSubsystem ionCannonSubsystem = new IonCannonSubsystem();
  //  private final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();
   private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
   private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
   private final TurretSubsystem turretSubsystem = new TurretSubsystem();
   private SendableChooser<Integer> autoCommandChooser = new SendableChooser<Integer>();
   private List<Trajectory> paths = new ArrayList<Trajectory>();
   private final Joystick manipulatorRightJoystick = new Joystick(2);
   private final Joystick manipulatorLeftJoystick = new Joystick(3);
   private final Joystick driverLeftJoystick = new Joystick(1);
   private final Joystick driverRightJoystick = new Joystick(0);


   public final HyperdriveSubsystem getHyperdriveSubsystem() {
      return this.hyperdriveSubsystem;
   }

   private final void configureButtonBindings() {
      new JoystickButton(this.driverRightJoystick, 1).whenPressed(new SwitchDriveDirection(this.hyperdriveSubsystem));
      new JoystickButton(this.driverLeftJoystick, 1).whileHeld(new RunDockingBay(this.dockingBaySubsystem, new PowerDistributionPanel(), -0.35D));
      new JoystickButton(this.driverLeftJoystick, 3).whileHeld(new RunDockingBay(this.dockingBaySubsystem, new PowerDistributionPanel(), 0.5D));
      new JoystickButton(this.driverRightJoystick,5).whenPressed(new InstantCommand(hyperdriveSubsystem::resetOdo,this.hyperdriveSubsystem));
      new JoystickButton(this.driverLeftJoystick,8).whenPressed(new InstantCommand(hyperdriveSubsystem::toggleIdle,this.hyperdriveSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 1).whileHeld(new TestShooterNT(this.ionCannonSubsystem, this.turboLiftSubsystem));
      // new JoystickButton(this.manipulatorRightJoystick, 5).whileHeld(new PrintColorSensorCommand(this.colorSensorSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 7).whileHeld(new RunColorMotor(this.controlPanelSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 6).whileHeld(new RunColorMotorOut(this.controlPanelSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 9).whileHeld(new Winch(this.climbSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 10).whileHeld(new HookDown(this.climbSubsystem));
      new JoystickButton(this.manipulatorRightJoystick, 11).whileHeld(new HookUp(this.climbSubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 1).whenPressed(new ReadyIonCannon(this.ionCannonSubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 2).whileHeld(new LaunchIonCannon(136000.0D, 134000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 3).whileHeld(new LaunchIonCannon(138000.0D, 136000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 4).whileHeld(new LaunchIonCannon(140000.0D, 138000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 5).whileHeld(new LaunchIonCannon(142000.0D, 140000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 6).whileHeld(new LaunchIonCannon(143500.0D, 143000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 7).whileHeld(new LaunchIonCannon(155000.0D, 156000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 8).whileHeld(new LaunchIonCannon(158000.0D, 160000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 9).whileHeld(new LaunchIonCannon(164000.0D, 165000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 10).whileHeld(new TurretSeekLimelight(this.turretSubsystem));
      new JoystickButton(this.manipulatorLeftJoystick, 11).whileHeld(new LaunchIonCannon(80000.0D, 100000.0D, this.ionCannonSubsystem, this.turboLiftSubsystem, this.manipulatorLeftJoystick, this.dockingBaySubsystem));
   }

 
   public final Command getAutonomousCommand() {
      int trajectoryIdx = this.autoCommandChooser.getSelected();

      Trajectory trajectory = paths.get(trajectoryIdx);

      if (trajectoryIdx == 3) {
        return CommandGroupBase.deadline(
          new r2goBrrrrr(this.hyperdriveSubsystem, trajectory),
          new RunDockingBay(this.dockingBaySubsystem, new PowerDistributionPanel(), -0.35D));
      } else if(trajectoryIdx == 1) {
        return new r2goBounce(this.hyperdriveSubsystem, trajectory, paths.get(5), paths.get(6), paths.get(7) );
      } else {
        return new r2goBrrrrr(this.hyperdriveSubsystem, trajectory);
      }
   }

   public RobotContainer() {
     System.out.println("Starting Robot Container");
      this.configureButtonBindings();
      this.autoCommandChooser.addOption("Slalom", 0);
      this.autoCommandChooser.addOption("Bounce", 1);
      this.autoCommandChooser.addOption("Barrel", 2);
      this.autoCommandChooser.addOption("Galactic Search", 3);
      this.autoCommandChooser.setDefaultOption("Test", 4);
      String[] files = {"Slalom.wpilib.json", "Bounce1.wpilib.json", "NewBarrel.wpilib.json", "GalacticSearch.wpilib.json", "Test2.wpilib.json", "Bounce2.wpilib.json", "Bounce3.wpilib.json","Bounce4.wpilib.json"};
      for (int i = 0; i < files.length; i++) {
        String trajectoryJSON = "paths/output/" + files[i];
        Trajectory trajectory = new Trajectory();
        try {
          System.out.println("Getting " + files[i]);
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          System.out.println("Unable to load file" + files[i]);
        }  
        List<Trajectory.State> states = trajectory.getStates();
        Trajectory.State lastState = states.get(states.size()-1);
        lastState.accelerationMetersPerSecondSq = 0.0;
        lastState.curvatureRadPerMeter = 0.0;
        lastState.velocityMetersPerSecond = 0.0;
        this.paths.add(trajectory);
      }

      SmartDashboard.putData("Auto mode", (Sendable)this.autoCommandChooser);
      SmartDashboard.putNumber("topShooterSpeed", 208000.0D);
      SmartDashboard.putNumber("bottomShooterSpeed", 208000.0D);

      this.hyperdriveSubsystem.setDefaultCommand(new DriveHyperCommand(this.hyperdriveSubsystem, this.driverLeftJoystick, this.driverRightJoystick));
      this.turboLiftSubsystem.setDefaultCommand(new TurboLiftDefault(this.turboLiftSubsystem, this.manipulatorRightJoystick));
      this.turretSubsystem.setDefaultCommand(new TurretManualDrive(this.turretSubsystem, this.manipulatorLeftJoystick));
      // CameraServer.getInstance().startAutomaticCapture();
      System.out.println("End of Robot Container Constructor");
   }
}
