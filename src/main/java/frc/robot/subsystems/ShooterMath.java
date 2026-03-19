package frc.robot.subsystems;
import static frc.robot.Constants.Vision.kTagLayout;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.tools.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;


public class ShooterMath {

    private static final double kickerMultiplier = 1.7;

    // Distance (meters) → RPM
    private static final InterpolatingDoubleTreeMap rpmMap = 
        new InterpolatingDoubleTreeMap();

    private static final double SHOOTER_HEIGHT = 0.3175; // meters
    private static final double HUB_HEIGHT = 1.8288;     // meters
    private static final double HUB_RADIUS = 0.5969;     // meters

    private static final double WHEEL_DIAMETER = 0.102;  // meters
    private static final double MOTOR_TO_WHEEL_RATIO = 36.0 / 15.0;
    
        static {
            // ----- TUNE THESE VALUES -----
            // distance , RPM
           rpmMap.put(2.159,  906.75);   // 906.75 * 1.10
            rpmMap.put(2.413, 930.0);    // 930 * 1.10
            rpmMap.put(2.54, 975.0);     // 975 * 1.10
            rpmMap.put(2.667, 980.0);    // 980 * 1.10
            rpmMap.put(2.794, 985.0);    // 985 * 1.10
            rpmMap.put(3.048, 1025.0);    // 1025 * 1.10
            rpmMap.put(3.302, 1050.0);    // 1050 * 1.10
            rpmMap.put(3.556, 1090.0);    // 1090 * 1.10
            rpmMap.put(3.81, 1100.0);     // 1100 * 1.10
            rpmMap.put(3.937, 1125.5);    // 1125 * 1.10
            rpmMap.put(4.191, 1175.0);    // 1175 * 1.10
        }

        public static double getShooterRPM(double distance){

            double unclampedRPM = rpmMap.get(distance);

            double clampedRPM = MathUtil.clamp(unclampedRPM, 800, 6000);

            return clampedRPM;
        }
    
        public static double getShooterRPMWhileMoving(CommandSwerveDrivetrain swerve) {

            int tagId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;

            Pose2d robotPose = swerve.getState().Pose;

            ChassisSpeeds speeds = swerve.getState().Speeds;

            Translation2d hubTranslation = FieldUtil.getHubTranslation(kTagLayout, tagId).get();

            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotPose.getRotation());

            Translation2d velocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

            double distance = hubTranslation.getDistance(robotPose.getTranslation());

            double ballSpeed = rpmMap.get(distance) / 60.0 * Math.PI * WHEEL_DIAMETER;

            double timeOfFlight = distance / ballSpeed;

            Translation2d futurePose = robotPose.getTranslation().plus(velocity.times(timeOfFlight));

            double futureDistance = hubTranslation.getDistance(futurePose);

            Translation2d toHub = hubTranslation.minus(robotPose.getTranslation());

            Translation2d direction = toHub.div(toHub.getNorm());

            double towardHubVelocity = velocity.getX() * direction.getX() + velocity.getY() * direction.getY();

            double rpmAdjustment = towardHubVelocity * 120;

            double unclampedRPM = rpmMap.get(futureDistance) - rpmAdjustment;

            double clampedRPM = MathUtil.clamp(unclampedRPM, 800, 6000);

            return clampedRPM; 
        }

    public static double getShotTolerance(double distance) {
    double targetRadius = 0.2; // meters
    return Math.atan2(targetRadius, distance);
}

    public static double getBestShooterRPM(CommandSwerveDrivetrain swerve, Vision vision){
    double rpm;
    double speed = Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond);

    double distance = vision.getBestHubDistance();

     if(speed > 0.5) {
     rpm = ShooterMath.getShooterRPMWhileMoving(swerve);
     } else {
     rpm = ShooterMath.getShooterRPM(distance);
     }

     return MathUtil.clamp(rpm, 800, 6000);
    }

    public static double getBestKickerRPM(CommandSwerveDrivetrain swerve, Vision vision){
    double rpm;
    double speed = Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond);

    double distance = vision.getBestHubDistance();

     if(speed > 0.5) {
     rpm = ShooterMath.getShooterRPMWhileMoving(swerve);
     } else {
     rpm = ShooterMath.getShooterRPM(distance);
     }

     double unclampedRPM = rpm * kickerMultiplier;

    return MathUtil.clamp(unclampedRPM, 800, 6000);
    
    }
}

    
