package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.tools.CalcFromVision;

public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision vision;
    private final CalcFromVision calc;
    private final CommandSwerveDrivetrain swerve;



    public ShootBall(Shooter shooter, Vision vision, CommandSwerveDrivetrain swerve) {
        this.shooter = shooter;
        this.vision = vision;
        this.swerve = swerve;
        this.calc = new CalcFromVision(vision);
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {
      //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      //double angleRad = Math.toRadians(angleDeg);
     // double rpm = SmartDashboard.getNumber("SetRPM", 0);
     // double angle = SmartDashboard.getNumber("SetDegrees", 0);

     double rpm;
     double distance;

     Optional <Double> rawDistance = vision.getRawDistanceShooterToHubAnyTag();
     Optional <Double> poseDistance = vision.distanceToHubFromPose();

     if(!(rawDistance).isEmpty()){
        distance = rawDistance.get();
        SmartDashboard.putBoolean("Vision Calced rpm", true);
     }
     else if(!(poseDistance).isEmpty()){
        distance = poseDistance.get();
        SmartDashboard.putBoolean("Vision Calced rpm", true);
     }
     else{
        System.out.print("VISION FAILED default default distance set to 2m");
        SmartDashboard.putBoolean("Vision Calced rpm", false);
        distance = 2.0; // default distance is 2 meters 
     }

     double speed = Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond);

     if(speed > 0.5) {
     rpm = ShooterMath.getShooterRPMWhileMoving(swerve);
     } else {
     rpm = ShooterMath.getShooterRPM(distance);
     }
    //    double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
    //   double rpm = ShooterMath.getRPM(distance);
        shooter.setRPM(rpm);
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }
}