package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class PointAndRotate extends CommandBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    private final double kP = 0.5;
    private final double rotTolerance = Math.toRadians(5);

    private int primaryTag;
    private int secondaryTag;

    public PointAndRotate(
        CommandSwerveDrivetrain drivetrain,
        Vision vision
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;

        primaryTag   = isRed ? 10 : 26;
        secondaryTag = isRed ? 9  : 25;
    }

    @Override
    public void execute() {

        // Grab rotation errors & optional distances
        Optional<Rotation2d> primaryError   = vision.getRotationErrorToTag(primaryTag);
        Optional<Rotation2d> secondaryError = vision.getRotationErrorToTag(secondaryTag);

        Rotation2d rotError;
        double distanceToTag;

        if (primaryError.isPresent()) {
            rotError = primaryError.get();
            distanceToTag = vision.distanceToTagFromPose(primaryTag).orElse(Double.NaN);

        } else if (secondaryError.isPresent()) {
            rotError = secondaryError.get();
            distanceToTag = vision.distanceToTagFromPose(secondaryTag).orElse(Double.NaN);

        } else {
            drivetrain.stop();
            return;
        }

        double errorRad = rotError.getRadians();

        SmartDashboard.putNumber("Rotation Error (deg)", Math.toDegrees(errorRad));
        SmartDashboard.putNumber("Distance to Target (m)", distanceToTag);

        if (Math.abs(errorRad) < rotTolerance) {
            drivetrain.stop();
            return;
        }

        double turnPower = kP * errorRad;

        SwerveRequest request = new SwerveRequest().withXSpeed(0).withYSpeed(0).withRotationalRate(turnPower);

        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}