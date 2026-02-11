package frc.robot.subsystems;
import java.lang.Math;


public class ShooterMath {

    private static final double g = 9.81;

    private static final double SHOOTER_HEIGHT = 0.3175; // meters
    private static final double HUB_HEIGHT = 1.8288;     // meters
    private static final double HUB_RADIUS = 0.5969;     // meters

    private static final double WHEEL_DIAMETER = 0.102;  // meters
    private static final double MOTOR_TO_WHEEL_RATIO = 36.0 / 15.0;

    public static double calcVelocity(double distance, double theta) {
        double adjustedDistance = distance + HUB_RADIUS;
        double deltaH = HUB_HEIGHT - SHOOTER_HEIGHT;

        double numerator = g * Math.pow(adjustedDistance, 2);
        double denominator =
            2 * Math.pow(Math.cos(theta), 2) *
            (adjustedDistance * Math.tan(theta) - deltaH);

        double term = numerator / denominator;

        if (term <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(term);
    }

    public static double calcMotorRPM(double velocity) {
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;

        double flywheelRPM = (velocity / wheelCircumference) * 60.0;
        return flywheelRPM * MOTOR_TO_WHEEL_RATIO;
    }

    
}
