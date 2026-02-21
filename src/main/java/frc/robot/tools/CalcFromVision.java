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
        Optional<Double> primaryDist = vision.distanceToTagFromPose(primaryTag);

        Optional<Double> secondaryDist = vision.distanceToTagFromPose(secondaryTag);

        Optional<Double> chosenDist = primaryDist.isPresent() ? primaryDist : secondaryDist;

        if (chosenDist.isEmpty()) return Optional.empty();

        SmartDashboard.putNumber("vision Distance ", chosenDist.get());

        double distance = chosenDist.get() + .6;
        double rpm = ShooterMath.getRPM(distance);
        return Optional.of(rpm);
    }

}
