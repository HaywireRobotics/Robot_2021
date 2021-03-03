package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.JSONPlotter;
// import frc.robot.JSONPlotterNT;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public final class IonCannonSubsystem extends SubsystemBase {

   private final WPI_VictorSPX top = new WPI_VictorSPX(0);

   private final WPI_VictorSPX bottom = new WPI_VictorSPX(3);
   private final Encoder topEncoder;
   private final Encoder bottomEncoder;
   private double topEncoderRate;
   private double bottomEncoderRate;
   private final PIDController topPIDController;
   private final PIDController bottomPIDController;
   // private final JSONPlotter topJSONPlotter;
   // private final JSONPlotter bottomJSONPlotter;
   // private final JSONPlotter topAverageJSONPlotter;
   // private final JSONPlotter bottomAverageJSONPlotter;
   // private final JSONPlotterNT jsonPlotterNT;
   private double bottomSetpoint;
   private double topSetpoint;
   private final int pointsUntilReady;
   private final double marginOfError;
   private List<Double> topLastData;
   private List<Double> bottomLastData;
   private double topAverage;
   private double botAverage;
   public final WPI_VictorSPX getTop() {
      return this.top;
   }

   public final WPI_VictorSPX getBottom() {
      return this.bottom;
   }

   public void periodic() {
      this.topEncoderRate = this.topEncoder.getRate();
      this.bottomEncoderRate = this.bottomEncoder.getRate();
      this.topLastData = this.addDatapoint(this.topLastData, this.topEncoderRate);
      this.bottomLastData = this.addDatapoint(this.bottomLastData, this.bottomEncoderRate);
   }

   public final void resetPID() {
      this.topPIDController.reset();
      this.bottomPIDController.reset();
      // this.topJSONPlotter.resetCapture();
      // this.bottomJSONPlotter.resetCapture();
      // this.topAverageJSONPlotter.resetCapture();
      // this.bottomAverageJSONPlotter.resetCapture();
      // this.topJSONPlotter.recordSetpoint((Number)this.topSetpoint);
      // this.bottomJSONPlotter.recordSetpoint((Number)this.bottomSetpoint);
      // this.topAverageJSONPlotter.recordSetpoint((Number)this.topSetpoint);
      // this.bottomAverageJSONPlotter.recordSetpoint((Number)this.bottomSetpoint);
   }

   public final void endPID() {
      this.top.set(0.0D);
      this.bottom.set(0.0D);
      // this.jsonPlotterNT.publishJSONsToNT(CollectionsKt.listOf(new String[]{this.topJSONPlotter.getJSON(), this.bottomJSONPlotter.getJSON(), this.topAverageJSONPlotter.getJSON(), this.bottomAverageJSONPlotter.getJSON()}));
      // String var1 = "TOP JSON";
      // boolean var2 = false;
      // System.out.println(var1);
      // this.topJSONPlotter.outputDataAsJSON();
      // var1 = "BOTTOM JSON";
      // var2 = false;
      // System.out.println(var1);
      // this.bottomJSONPlotter.outputDataAsJSON();
   }

   public final void setSetpoints(Double l_bottomSetpoint, Double l_topSetpoint) {
      this.bottomSetpoint = l_bottomSetpoint;
      this.topSetpoint = l_topSetpoint;
   }

   public final void runPID() { 
      this.topUseOutput(this.topPIDController.calculate(this.topEncoder.getRate(), this.topSetpoint));
      this.bottomUseOutput(this.bottomPIDController.calculate(this.bottomEncoder.getRate(), this.bottomSetpoint));
   }

   private final void topUseOutput(double output) {
      double vtarget = (-output-0.3)*12;
      this.top.setVoltage(vtarget);
      SmartDashboard.putNumber("Top Cannon", vtarget );
      SmartDashboard.putNumber("Top Cannon Avg", this.topAverage );
      // this.topJSONPlotter.recordPoint((Number)this.topEncoderRate);
      // this.topAverageJSONPlotter.recordPoint((Number)CollectionsKt.averageOfDouble((Iterable)this.topLastData));
   }

   private final void bottomUseOutput(double output) {
      double vtarget = (-output-0.6)*12;
      this.bottom.setVoltage(vtarget);
      //this.bottom.setVoltage(0D);
      SmartDashboard.putNumber("Bottom Cannon", vtarget );
      SmartDashboard.putNumber("Bottom Cannon Avg", this.botAverage );
      // this.bottomJSONPlotter.recordPoint((Number)this.bottomEncoderRate);
      // this.topAverageJSONPlotter.recordPoint((Number)CollectionsKt.averageOfDouble((Iterable)this.bottomLastData));
   }

   public final boolean isReady() {
      double topAverageValue = calculateAverage(this.topLastData);
      double bottomAverageValue = calculateAverage(this.bottomLastData);
      boolean topOK = topAverageValue >= this.topSetpoint - this.marginOfError && topAverageValue <= this.topSetpoint + this.marginOfError;
      boolean bottomOK = bottomAverageValue >= this.bottomSetpoint - this.marginOfError && bottomAverageValue <= this.bottomSetpoint + this.marginOfError;
      this.topAverage = topAverageValue;
      this.botAverage = bottomAverageValue;
      return topOK && bottomOK;
   }

   private double calculateAverage(List <Double> marks) {
      double sum = 0;
      if(!marks.isEmpty()) {
        for (Double mark : marks) {
            sum += mark;
        }
        return sum / marks.size();
      }
      return sum;
    }
   private final List<Double> addDatapoint(List<Double> list, Double datapoint) {
      list.add(datapoint.doubleValue());
      if (list.size() <= this.pointsUntilReady) {
         return list;
      } else {
         list.remove(0);
         return list;
      }
   }

   public IonCannonSubsystem() {
      this.topEncoder = new Encoder(7, 6, false, EncodingType.k4X);
      this.bottomEncoder = new Encoder(9, 8, false, EncodingType.k4X);
      // this.topPIDController = new PIDController(56.25E-7D*3, 0D, 0D);
      // this.bottomPIDController = new PIDController(56.25E-7D*3, 0D, 0D);
      // this.topPIDController = new PIDController(5.625E-6D, 1.175E-5D, 1.75E-7D);
      this.topPIDController = new PIDController(5.625E-6D, 1.175E-5D/2, 1.75E-7D);
      // this.bottomPIDController = new PIDController(5.625E-6D, 1.175E-5D, 1.75E-7D);
      this.bottomPIDController = new PIDController(5.625E-6D, 1.175E-5D/2, 1.75E-7D);
      
      // this.topJSONPlotter = new JSONPlotter("Ion Top");
      // this.bottomJSONPlotter = new JSONPlotter("Ion Bottom");
      // this.topAverageJSONPlotter = new JSONPlotter("Top Average");
      // this.bottomAverageJSONPlotter = new JSONPlotter("Bottom Average");
      // this.jsonPlotterNT = new JSONPlotterNT();
      this.pointsUntilReady = 30;
      this.marginOfError = 5000.0D;
      // boolean var1 = false;
      this.topLastData = (List<Double>)(new ArrayList<Double>());
      // var1 = false;
      this.bottomLastData = (List<Double>)(new ArrayList<Double>());
      this.topEncoder.setDistancePerPulse(1.0D);
      // this.topEncoder.setPIDSourceType(PIDSourceType.kRate);
      this.bottomEncoder.setDistancePerPulse(1.0D);
      // this.bottomEncoder.setPIDSourceType(PIDSourceType.kRate);
   }
}
