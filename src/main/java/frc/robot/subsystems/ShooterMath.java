package frc.robot.subsystems;
import java.lang.Math;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.*;


public class ShooterMath {

    private static final double g = 9.81;

    // Distance (meters) → RPM
    private static final InterpolatingDoubleTreeMap rpmMap = 
        new InterpolatingDoubleTreeMap();

    // Distance (meters) → Hood Angle (degrees)
    private static final InterpolatingDoubleTreeMap hoodMap = 
        new InterpolatingDoubleTreeMap();


    private static final double SHOOTER_HEIGHT = 0.3175; // meters
    private static final double HUB_HEIGHT = 1.8288;     // meters
    private static final double HUB_RADIUS = 0.5969;     // meters

    private static final double WHEEL_DIAMETER = 0.102;  // meters
    private static final double MOTOR_TO_WHEEL_RATIO = 36.0 / 15.0;

    // public static double calcVelocity(double distance, double theta) {
    //     double adjustedDistance = distance + HUB_RADIUS;
    //     double deltaH = HUB_HEIGHT - SHOOTER_HEIGHT;

    //     double numerator = g * Math.pow(adjustedDistance, 2);
    //     double denominator =
    //         2 * Math.pow(Math.cos(theta), 2) *
    //         (adjustedDistance * Math.tan(theta) - deltaH);

    //     double term = numerator / denominator;

    //     return Math.sqrt(term);
    // }

    // public static double calcMotorRPM(double velocity) {
    //     double wheelCircumference = Math.PI * WHEEL_DIAMETER;

    //     double flywheelRPM = (velocity / wheelCircumference) * 60.0;
    //     return flywheelRPM * MOTOR_TO_WHEEL_RATIO;
    // }

    static {
        // ----- TUNE THESE VALUES -----
        // distance , RPM
        rpmMap.put(2.159, 997.425);   // 906.75 * 1.10
rpmMap.put(2.413, 1023.0);    // 930 * 1.10
rpmMap.put(2.54, 1072.5);     // 975 * 1.10
rpmMap.put(2.667, 1078.0);    // 980 * 1.10
rpmMap.put(2.794, 1083.5);    // 985 * 1.10
rpmMap.put(3.048, 1127.5);    // 1025 * 1.10
rpmMap.put(3.302, 1155.0);    // 1050 * 1.10
rpmMap.put(3.556, 1199.0);    // 1090 * 1.10
rpmMap.put(3.81, 1210.0);     // 1100 * 1.10
rpmMap.put(3.937, 1237.5);    // 1125 * 1.10
rpmMap.put(4.191, 1292.5);    // 1175 * 1.10


        // distance , hood angle (degrees)
        hoodMap.put(1.5, 15.0);
        hoodMap.put(2.0, 18.0);
        hoodMap.put(2.5, 21.0);
        hoodMap.put(3.0, 25.0);
    }

    public static double getRPM(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public static double getHoodAngle(double distanceMeters) {
        return hoodMap.get(distanceMeters);
    }
}

    
