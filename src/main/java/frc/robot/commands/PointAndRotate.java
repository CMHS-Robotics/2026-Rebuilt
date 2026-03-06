package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class PointAndRotate extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    private final double kP = 3;
    private final double rotTolerance = Math.toRadians(5);

    private int primaryTag;
    private int secondaryTag;

    private double lastValidErrorRad = 0;
    private int lostFramesCounter = 0;
    private final int MAX_LOST_FRAMES = 15; // About 0.3 seconds at 50Hz

    private final SwerveRequest zero = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    public PointAndRotate(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        boolean isRed = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;

        primaryTag   = isRed ? 10 : 26;
        secondaryTag = isRed ? 9  : 25;
    }

    @Override
    public void execute() {
        Optional<Rotation2d> primaryError   = vision.getRawRotationErrorToHubAnyTag(primaryTag);
        Optional<Rotation2d> secondaryError = vision.getDirectRotationErrorShooterToTag(secondaryTag);

        double errorRad;
        boolean canSeeTarget = primaryError.isPresent() || secondaryError.isPresent();

        if (canSeeTarget) {
            // Reset counter and update the "last known" error
            lostFramesCounter = 0;
            errorRad = primaryError.isPresent() ? 
                       primaryError.get().getRadians() : 
                       secondaryError.get().getRadians();
            lastValidErrorRad = errorRad;
        } else {
            // We lost vision! Increment counter
            lostFramesCounter++;
            
            if (lostFramesCounter <= MAX_LOST_FRAMES) {
                // Keep using the last known error to "coast" through the dropout
                errorRad = lastValidErrorRad;
            } else {
                // We've been blind for too long, safety stop
                drivetrain.setControl(zero);
                return;
            }
        }

        // Apply Deadband/Tolerance check
        if (Math.abs(errorRad) < rotTolerance) {
            drivetrain.setControl(zero);
            return;
        }

        // Standard PID logic using either fresh or "ghost" errorRad
        double turnPower = MathUtil.clamp(kP * errorRad, -3.0, 3.0);
        
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(turnPower));

        // Feedback for debugging
        SmartDashboard.putNumber("Lost Vision Frames", lostFramesCounter);
        SmartDashboard.putBoolean("Vision Active", canSeeTarget);
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(zero);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}