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

    

<<<<<<< Updated upstream
        boolean isRed = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);

        primaryTag   = isRed ? 10 : 26;
        secondaryTag = isRed ? 9  : 25;

       
        // Grab rotation errors & optional distances
        Optional<Double> primaryDist =   vision.distanceToTagFromPose(primaryTag);

         Optional<Double> secondaryDist = vision.distanceToTagFromPose(secondaryTag);

        Optional<Double> chosenDist = primaryDist.isPresent() ? primaryDist : secondaryDist;
        SmartDashboard.putNumber("vision Distance", chosenDist.get());
        if (chosenDist.isEmpty()) {
            SmartDashboard.putBoolean("Vision Distance Valid", false);
            return Optional.empty();

}



        double rawDistance = chosenDist.get(); // w/o hub offsett
        SmartDashboard.putBoolean("Vision Distance Valid", true);
        SmartDashboard.putNumber("Vision Raw Distance", rawDistance);

        double compedDistance = rawDistance + 0.6; // with hub offset
        double rpm = ShooterMath.getRPM(compedDistance);
        SmartDashboard.putNumber("Vision calc RPM", rpm);
=======
    public Optional<Double> calcHubRPM() {

        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);
    
        int centerTag = isRed ? 10 : 26;
        int leftTag   = isRed ? 9  : 25;
    
        // Try center tag first
        Optional<Translation2d> centerTranslation =
            vision.translationToTagFromPose(centerTag);
        
        Translation2d finalTranslation;
    
        if (centerTranslation.isPresent()) {
    
            finalTranslation = centerTranslation.get();
    
        } else {
    
            Optional<Translation2d> leftTranslation =
                vision.translationToTagFromPose(leftTag);
    
            if (leftTranslation.isEmpty()) {
                return Optional.empty();
            }
    
            // Offset 14in toward hub center
            // IMPORTANT: This assumes left tag is 14in left along HUB FACE
            Translation2d hubOffset = new Translation2d(0.3556, 0);
            finalTranslation = leftTranslation.get().plus(hubOffset);
        }
    
        double distance = finalTranslation.getNorm();
    
        SmartDashboard.putNumber("Vision Distance", distance);
    
        double adjustedDistance = distance + 0.6;
        double rpm = ShooterMath.getRPM(adjustedDistance);
    
>>>>>>> Stashed changes
        return Optional.of(rpm);
    }



}
