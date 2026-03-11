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
         if(!distance.isEmpty()){
            SmartDashboard.putNumber("Vision Distance", distance.get());
         }
         else{
             SmartDashboard.putNumber("Vision Distance", 0); 
         }
    
         double adjustedDistance = distance.get() + 0.6;
         double rpm = ShooterMath.getShooterRPM(adjustedDistance);
    
         return Optional.of(rpm);
         
     }

     public Optional<Double> calcHubRPMUsingPose(){

        int hubTag = getHubTagID();
    
        Optional<Double> distance = vision.distanceToTagFromPose(hubTag);
    
         SmartDashboard.putNumber("Vision Distance", distance.get());
    
         double adjustedDistance = distance.get() + 0.6;
         double rpm = ShooterMath.getShooterRPM(adjustedDistance);
    
         return Optional.of(rpm);
     }

     private int getHubTagID() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
               == DriverStation.Alliance.Red ? 10 : 26;
    }



}
