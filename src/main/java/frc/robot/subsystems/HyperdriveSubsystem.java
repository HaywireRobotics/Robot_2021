package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

//import jdk.vm.ci.meta.Constant;



public final class HyperdriveSubsystem extends SubsystemBase {
   private final CANSparkMax leftFront;
   private final CANSparkMax leftBack;
   private final CANSparkMax rightFront;
   private final CANSparkMax rightBack;
   private final DifferentialDrive myRobot;
   private boolean robotDirectionInverted;
   private final CANEncoder leftEncoder;
   private final CANEncoder rightEncoder;
   private final CANPIDController leftController;
   private final CANPIDController rightController;
   private final AHRS gyro = new AHRS();
   private final DifferentialDriveOdometry odometry;
   private boolean forward = true;
   public final boolean getRobotDirectionInverted() {
      return this.robotDirectionInverted;
   }

   public final void setRobotDirectionInverted(boolean var1) {
      this.robotDirectionInverted = var1;
   }

   public void periodic() {
      if (forward) {
         odometry.update(gyro.getRotation2d().times(1.0), leftEncoder.getPosition(), -rightEncoder.getPosition());
      } else {
         odometry.update(gyro.getRotation2d().times(1.0), rightEncoder.getPosition(), -leftEncoder.getPosition());
      }
      if (true) {
     double leftEncoderPosition = leftEncoder.getPosition();
     double rightEncoderPosition = rightEncoder.getPosition();
     double leftVelocity = leftEncoder.getVelocity();
     double rightVelocity = -rightEncoder.getVelocity();
     double leftLeadAppliedOutput = leftFront.getAppliedOutput();
     double leftFollowAppliedOutput = leftBack.getAppliedOutput();
     double rightLeadAppliedOutput = rightFront.getAppliedOutput();
     double rightFollowAppliedOutput = rightBack.getAppliedOutput();

      SmartDashboard.putNumber("Rotation", odometry.getPoseMeters().getRotation().getDegrees() );
      SmartDashboard.putNumber("Translation X", odometry.getPoseMeters().getTranslation().getX());
      // SmartDashboard.putNumber("Translation X", leftEncoder.getPosition());
      SmartDashboard.putNumber("Translation Y", odometry.getPoseMeters().getTranslation().getY());
      // SmartDashboard.putNumber("Translation Y", rightEncoder.getPosition());

      SmartDashboard.putNumber("Left Wheel Velocity", leftVelocity);
      SmartDashboard.putNumber("Right Wheel Velocity", rightVelocity);  

      SmartDashboard.putNumber("Gyro Position", gyro.getAngle());

   }}

   public Pose2d getPose() {
      return odometry.getPoseMeters();
   }

   public Translation2d getTranslation() {
      return odometry.getPoseMeters().getTranslation();
   }

   public final void tankDrive(double leftPower, double rightPower) {
      this.myRobot.tankDrive(-leftPower, -rightPower, true);
   }

   public void tankDriveVolts(double leftVolts, double rightVolts) {
      // System.out.println("Left: " + leftVolts + "  Right: " + rightVolts);

      leftFront.setVoltage(leftVolts);
      leftBack.setVoltage(leftVolts);
      rightFront.setVoltage(rightVolts);
      rightBack.setVoltage(rightVolts);
   }

   // returns the wheel speeds
   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
   }

   public void resetOdometry(Pose2d pose) {
      resetEncoders();
      gyro.reset();
      // gyro.setAngleAdjustment(adjustment);
      odometry.resetPosition(pose, gyro.getRotation2d());
      System.out.println("Resetting Odometry: " + odometry.getPoseMeters());
   }
   public void flipOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition(getPose(), gyro.getRotation2d().plus(new Rotation2d(3.14159268)));
      System.out.println("Offsetting Odometry: " + odometry.getPoseMeters());
   }

   public void resetOdo() {
      resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
   }

   public void zeroHeading() {
      gyro.reset();
   }

   public void resetEncoders() {
      leftEncoder.setPosition(0.0);
      rightEncoder.setPosition(0.0);
   }

   public double rightEncoderPos() {
      return rightEncoder.getPosition();
   }
   public double leftEncoderPos() {
      return leftEncoder.getPosition();
   }
   public void setWheelVelocity(double left, double right) {
      // System.out.println("left: " + left + "right" + right);
      forward = true;
      leftController.setReference(left, ControlType.kVelocity);
      rightController.setReference(-right, ControlType.kVelocity);
   }
   public void setReverseWheelVelocity(double left, double right) {
      // System.out.println("left: " + left + "right" + right);
      forward = false;
      leftController.setReference(-right, ControlType.kVelocity);
      rightController.setReference(left, ControlType.kVelocity);
   }
   public double getHeading() {
      return gyro.getRotation2d().times(-1.0).getDegrees();
   }

   public double getTurnRate() {
      return gyro.getRate();
   }

   public void toggleIdle() {
      if (leftFront.getIdleMode() == IdleMode.kBrake) {
         leftFront.setIdleMode(IdleMode.kCoast);
         leftBack.setIdleMode(IdleMode.kCoast);
         rightFront.setIdleMode(IdleMode.kCoast);
         rightBack.setIdleMode(IdleMode.kCoast);
      } else {
         leftFront.setIdleMode(IdleMode.kBrake);
         leftBack.setIdleMode(IdleMode.kBrake);
         rightFront.setIdleMode(IdleMode.kBrake);
         rightBack.setIdleMode(IdleMode.kBrake);
      }
   }
   public HyperdriveSubsystem() {
      this.leftFront = new CANSparkMax(Constants.Hyperdrive.leftFrontPort, MotorType.kBrushless);
      this.leftBack = new CANSparkMax(Constants.Hyperdrive.leftBackPort, MotorType.kBrushless);
      this.rightFront = new CANSparkMax(Constants.Hyperdrive.rightFrontPort, MotorType.kBrushless);
      this.rightBack = new CANSparkMax(Constants.Hyperdrive.rightBackPort, MotorType.kBrushless);
      this.leftBack.follow(this.leftFront);
      this.rightBack.follow(this.rightFront);
      this.leftController = this.leftFront.getPIDController();
      this.rightController = this.rightFront.getPIDController();

      this.leftController.setP(Constants.Hyperdrive.kPDriveVel);
      this.rightController.setP(Constants.Hyperdrive.kPDriveVel);
      this.leftController.setD(Constants.Hyperdrive.kD);
      this.rightController.setD(Constants.Hyperdrive.kD);
      this.leftController.setFF(0.000015*10*10*0.15);
      this.rightController.setFF(0.000015*10*10*0.15);

      this.myRobot = new DifferentialDrive(this.leftFront,this.rightFront);
      this.leftEncoder = this.leftFront.getEncoder();
      this.rightEncoder = this.rightFront.getEncoder();
      this.leftEncoder.setPositionConversionFactor(Constants.Hyperdrive.kEncoderDistancePerPulse);
      this.rightEncoder.setPositionConversionFactor(Constants.Hyperdrive.kEncoderDistancePerPulse);
      resetEncoders();
      this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d().times(-1.0));
   }
}
