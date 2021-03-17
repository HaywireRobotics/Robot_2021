package frc.robot;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


public final class Constants {
   
   public static final class Joysticks {
      public static final int driverRightPort = 0;
      public static final int driverLeftPort = 1;
      public static final int manipulatorRightPort = 2;
      public static final int manipulatorLeftPort = 3;
   }

   public static final class DockingBay {
      public static final int intakePort = 50;
      public static final int agitatorPort = 1;
   }
   
   public static final class Hyperdrive {
      public static final int rightFrontPort = 23;
      public static final int rightBackPort = 26;
      public static final int leftFrontPort = 21;
      public static final int leftBackPort = 22;
      public static final double ksVolts = 0.173;
      public static final double kvVoltsSecondsPerMeter = 2.78;
      public static final double kaVoltSecondsSquarePerMeter = 0.509;
      public static final double kPDriveVel = 0.00362/10 * 0.75; //0.0000238; //2.38;
      public static final double kD = 0.0;
      public static final double kTrackwidthMeters = 1.22388; // check here if circlessss
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
      public static final double kMaxSpeedMetersPerSecond = 1.5;
      public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
      //Encoder is 42 counts per rev.
      // each rev is 1/10.5 rotations of a tire
      // each tire is 6 inches diameter.
      //public static final double kEncoderDistancePerPulse = 0.0114; //this number MIGHT be VERY wrong :)
      // public static final double kEncoderDistancePerPulse = 0.478;
      public static final double kEncoderDistancePerPulse = (0.478 / 0.342) * 1.2*0.85694 * 1.125;   
   
   }

   public static final class TurboLift {
      public static final int backMotorPort = 2;
      public static final int frontMotorPort = 40;
   }
   
   public static final class ControlPanel {
      public static final int colorMotor = 11;
   }
  
   public static final class IonCannon {
      public static final int topPort = 0;
      public static final int bottomPort = 3;
      public static final int topEncoderAPort = 7;
      public static final int topEncoderBPort = 6;
      public static final int bottomEncoderAPort = 9;
      public static final int bottomEncoderBPort = 8; 
   }
   
   public static final class Climb {
      public static final int hookPort = 12;
      public static final int winchPort = 5; 
   }
   
   public static final class Turret {
      public static final int motorPort = 4;
   }

   public static final class Trajectory {

   }
}
