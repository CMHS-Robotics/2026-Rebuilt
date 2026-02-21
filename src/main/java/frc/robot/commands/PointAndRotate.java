package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

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
            drivetrain.setControl(zero);
            return;
        }

        double errorRad = rotError.getRadians();

        if (Math.abs(errorRad) < rotTolerance) {
            drivetrain.setControl(zero);
            return;
        }

        double turnPower = kP * errorRad;

         SwerveRequest request = new SwerveRequest.RobotCentric() //try robot centric if this is cooked
         .withVelocityX(0)
         .withVelocityY(0)
         .withRotationalRate(turnPower);

         drivetrain.setControl(request);

         SmartDashboard.putBoolean("Primary Seen", primaryError.isPresent());
         SmartDashboard.putBoolean("Secondary Seen", secondaryError.isPresent());
         SmartDashboard.putNumber("Rotation Error Deg", Math.toDegrees(errorRad));
         SmartDashboard.putNumber("Turn Power", turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> zero);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}