package frc.robot.tools;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CalcFromVision {

    private final Vision vision;

    public CalcFromVision(Vision vision) {
        this.vision = vision;
    }

     public Optional<Double> calcHubRPM() {

         boolean isRed = DriverStation.getAlliance()
             .map(a -> a == DriverStation.Alliance.Red)
             .orElse(false);
    
         int centerTag = isRed ? 10 : 26;
         int leftTag   = isRed ? 9  : 25;
    
         // Try center tag first
         Optional<Translation2d> centerTranslation = vision.getDirectTranslationShooterToTag(centerTag);
        
         Translation2d finalTranslation;
    
         if (centerTranslation.isPresent()) {
    
             finalTranslation = centerTranslation.get();
    
         } else {
    
             Optional<Translation2d> leftTranslation = vision.getDirectTranslationShooterToTag(leftTag); // can also use getDirectTranslationToTag for robot angling
    
             if (leftTranslation.isEmpty()) {
                 return Optional.empty();
             }
    
    //         // Offset 14in toward hub center
    //         // IMPORTANT: This assumes left tag is 14in left along HUB FACE
             Translation2d hubOffset = new Translation2d(0.3556, 0);
             finalTranslation = leftTranslation.get().plus(hubOffset);
         }
    
         double distance = finalTranslation.getNorm();
    
         SmartDashboard.putNumber("Vision Distance", distance);
    
         double adjustedDistance = distance + 0.6;
         double rpm = ShooterMath.getRPM(adjustedDistance);
    
         return Optional.of(rpm);
     }


     public Optional<Double> calcHubRPMUsingDistance() { //in case the hubOffset is wrong for left tag 

        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);
    
        int centerTag = isRed ? 10 : 26;
    
        // only center
        Optional<Double> distanceToCenter = vision.getDirectDistanceToTag(centerTag);

        Double finalDistance;
    
        if (distanceToCenter.isPresent()) {
    
            finalDistance = distanceToCenter.get();
    
        } else {
            return Optional.empty();
        }
    
        double distance = finalDistance;
    
        SmartDashboard.putNumber("Vision Distance", distance);
    
        double adjustedDistance = distance + 0.6;
        double rpm = ShooterMath.getRPM(adjustedDistance);
    
        return Optional.of(rpm);
    }



}
