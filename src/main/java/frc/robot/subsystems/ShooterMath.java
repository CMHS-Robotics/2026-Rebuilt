package frc.robot.subsystems;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.lang.Math;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterMath {

public static double calcVelocity(double distance, double theta){
    // Calculate the required velocity to shoot a projectile to a target at a given distance and angle
    // Using the formula: v = sqrt((d * g) / sin(2 * theta))
    double g = 9.81; // Acceleration due to gravity in m/s^2
    double h = 0.3175; //height of bot aprox needs to be adjusted
    distance = distance + .5969; // account for the length of the hub 
    double numerator = (g * Math.pow(distance,2));
    double denomonator = 2 * Math.pow((Math.cos(theta)),2) * (distance * Math.tan(theta) - (1.8288 - h)); //theta must be in radians
    double velocity = Math.sqrt(- numerator / denomonator);
    return velocity;
}

public static double calcRPM(double velocity){
    // Convert linear velocity (m/s) to rotational speed (RPM)
    double wheelDiameter = 0.102; // Diameter of the shooter wheel in meters (example value)
    double wheelCircumference = Math.PI * wheelDiameter; // Circumference of the wheel
    double rpm = (velocity / wheelCircumference) * 60; // Convert m/s to RPM
    return rpm;
}
}
