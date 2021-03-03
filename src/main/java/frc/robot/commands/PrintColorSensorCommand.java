package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ColorSensorSubsystem;



public final class PrintColorSensorCommand extends CommandBase {
   private final ColorSensorSubsystem m_subsystem;

   public void initialize() {
   }

   public void execute() {
      Color color = this.m_subsystem.getColorRGB();
      double var2 = color.red;
      // boolean var4 = false;
      System.out.println(var2);
      var2 = color.green;
      // var4 = false;
      System.out.println(var2);
      var2 = color.blue;
      // var4 = false;
      System.out.println(var2);
      // boolean var5 = false;
      System.out.println();
   }

   public void end(boolean interrupted) {
   }

   public boolean isFinished() {
      return false;
   }

   public final ColorSensorSubsystem getM_subsystem() {
      return this.m_subsystem;
   }

   public PrintColorSensorCommand(ColorSensorSubsystem m_subsystem) {
      super();
      this.m_subsystem = m_subsystem;
      this.addRequirements(new Subsystem[]{(Subsystem)this.m_subsystem});
   }
}
