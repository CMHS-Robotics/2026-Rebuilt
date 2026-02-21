package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision vision;


    public ShootBall(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {

       Optional<Double> primaryDistance   = vision.distanceToTagFromPose(10);
       Optional<Double> secondaryDistance = vision.distanceToTagFromPose(9);

        
        double distance;

        if (primaryDistance.isPresent()) {
            distance = vision.distanceToTagFromPose(10).orElse(Double.NaN);

        } else if (secondaryDistance.isPresent()) {
            distance = vision.distanceToTagFromPose(9).orElse(Double.NaN);

        } else {

            return;
        }
      //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      //double angleRad = Math.toRadians(angleDeg);
      //double velocity = ShooterMath.calcVelocity(distance, angleRad);
      //double rpm = ShooterMath.calcMotorRPM(velocity);

      // double rpm = ShooterMath.getRPM(distance);
     // double rpm = SmartDashboard.getNumber("SetRPM", 0);
     // double angle = SmartDashboard.getNumber("SetDegrees", 0);
     double rpm = ShooterMath.getRPM(distance);
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