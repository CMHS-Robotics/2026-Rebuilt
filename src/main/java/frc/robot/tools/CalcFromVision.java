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

     public Optional<Double> calcHubRPMUsingAnyTag(){
         Optional<Double> distance = vision.getRawDistanceShooterToHubAnyTag();
    
         SmartDashboard.putNumber("Vision Distance", distance.get());
    
         double adjustedDistance = distance.get() + 0.6;
         double rpm = ShooterMath.getRPM(adjustedDistance);
    
         return Optional.of(rpm);
         
     }

     public Optional<Double> calcHubRPMUsingPose(){

        int hubTag;

        if(DriverStation.getAlliance().get().equals("red")){
            hubTag = 10;
        }
        else {
            hubTag = 26;
        }
    
        Optional<Double> distance = vision.distanceToTagFromPose(hubTag);
    
         SmartDashboard.putNumber("Vision Distance", distance.get());
    
         double adjustedDistance = distance.get() + 0.6;
         double rpm = ShooterMath.getRPM(adjustedDistance);
    
         return Optional.of(rpm);
     }



}
