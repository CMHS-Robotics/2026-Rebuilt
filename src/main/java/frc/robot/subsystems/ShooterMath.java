package frc.robot.subsystems;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import Math;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterMath {

double calcVelocity(double distance, double theta){
    // Calculate the required velocity to shoot a projectile to a target at a given distance and angle
    // Using the formula: v = sqrt((d * g) / sin(2 * theta))
    double g = 9.81; // Acceleration due to gravity in m/s^2
    double h = 0.5334 //height of bot aprox needs to be adjusted
    distance = distance + .5969; // account for the length of the hub 
    double numerator = (g * Math.pow(d,2));
    double denomonator = 2 * Math.pow((Math.cos(theta)),2) * (distance * Math.tan(theta) - (1.8288 - h)); //theta must be in radians
    double velocity = Math.sqrt(- numerator / denomonator);
    return velocity;
}

double calcRPM(double velocity){
    // Convert linear velocity (m/s) to rotational speed (RPM)
    double wheelDiameter = 0.1016; // Diameter of the shooter wheel in meters (example value)
    double wheelCircumference = Math.PI * wheelDiameter; // Circumference of the wheel
    double rpm = (velocity / wheelCircumference) * 60; // Convert m/s to RPM
    return rpm;
}

Optional<Transform3d> getShooterTransform(){
    // Placeholder for getting the best target transform from vision system
    Transform3d RobotToShooterTransform = new Transform3d(0,0,0,new (0,0,0)); // Replace with actual transform retrieval logic once measured, this is to the center of the shooter

    //return Optional.empty(); gotta fix this optional return, honestly method may not be needed
}

public void aimAndShoot() { // need to update vision to match this format, pose from odometry fused with vision rather than just vision pose
    Pose2d robotPose = swerve.getPose();
    Pose2d shooterPose = robotPose.transformBy(getShooterTransform()));

    Translation2d toHub =
        FieldConstants.HUB_POSE.getTranslation()
            .minus(shooterPose.getTranslation());

    Rotation2d targetHeading = toHub.getAngle();
    double distance = toHub.getNorm();

    // Rotate robot
    swerve.turnToHeading(targetHeading);

    // Shooter math
    double rpm = ShooterMath.calculateRPM(distance);
    shooter.setRPM(rpm);

    boolean aimed =
        Math.abs(
            targetHeading.minus(robotPose.getRotation()).getDegrees()
        ) < 1.0;

    if (aimed && shooter.atSetpoint()) {
        feeder.feed();
    }
}

public void setRPM(double RPM){
    
}
