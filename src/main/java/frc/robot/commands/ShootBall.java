package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.tools.CalcFromVision;
import frc.robot.tools.FuelSim;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision vision;
    private final CalcFromVision calc;
    private final CommandSwerveDrivetrain swerve;
    private final FuelSim fs;
    private boolean hasFired = false;



    public ShootBall(Shooter shooter, Vision vision, CommandSwerveDrivetrain swerve, FuelSim fs) {
        this.shooter = shooter;
        this.vision = vision;
        this.swerve = swerve;
        this.calc = new CalcFromVision(vision);
        this.fs = fs;
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
        hasFired = false;
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

   if (!hasFired) {
        double velocityMPS = (rpm * 2 * Math.PI * Units.inchesToMeters(4)) / 60.0;
        double exitVelocity = velocityMPS * 0.7; 

        fs.launchFuel(
            MetersPerSecond.of(exitVelocity),
            Degrees.of(70), 
            Degrees.of(0.0),  
            Meters.of(0.8)    
        );
        hasFired = true; // Prevents spawning 50 balls per second
    }
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