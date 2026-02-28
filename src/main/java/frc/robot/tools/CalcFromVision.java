package frc.robot.tools;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CalcFromVision {

    private final Vision vision;
    private int primaryTag;
    private int secondaryTag;

    public CalcFromVision(Vision vision){
        this.vision = vision;
    }

    public Optional<Double> calcHubRPM(){

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
        return Optional.of(rpm);
    }

}
