package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.tools.CalcFromVision;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Kick extends Command{
    private final Kicker kicker;
    private final Vision vision;
    private final CalcFromVision calc;
    private final CommandSwerveDrivetrain swerve;

    public Kick(Kicker kicker, Vision vision, CommandSwerveDrivetrain swerve) {
        this.kicker = kicker;
        this.vision = vision;
        this.swerve = swerve;
        this.calc = new CalcFromVision(vision);
        addRequirements(kicker); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        kicker.resetRamp();
    }

    @Override
    public void execute() {
         //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
         //double angleRad = Math.toRadians(angleDeg);
    
         // double rpm = ShooterMath.getRPM(distance);
       //  double rpm = SmartDashboard.getNumber("SetRPM", 0);
     double rpm;
     double distance;

     Optional <Double> rawDistance = vision.getRawDistanceShooterToHubAnyTag();
     Optional <Double> poseDistance = vision.distanceToHubFromPose();

     if(!(rawDistance).isEmpty()){
        distance = rawDistance.get();
        SmartDashboard.putBoolean("Vision Calced Kicker distance", true);
        SmartDashboard.putNumber("Vision distance", distance);
     }
     else if(!(poseDistance).isEmpty()){
        distance = poseDistance.get();
        SmartDashboard.putBoolean("Vision Calced Kicker distance", true);
        SmartDashboard.putNumber("Vision distance", distance);
     }
     else{
        System.out.print("VISION FAILED default default distance set to 2m");
        SmartDashboard.putBoolean("Vision Calced Kicker distance", false);
        distance = 2.0; // default distance is 2 meters 
        SmartDashboard.putNumber("Vision distance", distance);
     }

     double speed = Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond);

     if(speed > 0.5) {
     rpm = ShooterMath.getKickerRPMWhileMoving(swerve);
     } else {
     rpm = ShooterMath.getKickerRPM(distance);
     }
    //    double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
    //   double rpm = ShooterMath.getRPM(distance);
        kicker.setRPM(-rpm);
     // double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
     //  double rpm = ShooterMath.getRPM(distance);
     //   kicker.setRPM(-rpm);
    }

    @Override
    public void end(boolean interrupted) {
      kicker.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }

}
