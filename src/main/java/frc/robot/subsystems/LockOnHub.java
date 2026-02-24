import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class LockOnHub extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout layout;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle();

    public LockOnHub(
        CommandSwerveDrivetrain drivetrain,
        AprilTagFieldLayout layout,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        this.drivetrain = drivetrain;
        this.layout = layout;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getPose();

        int tagID = getHubTagID();

        Optional<Pose2d> tagPoseOpt =
            layout.getTagPose(tagID).map(p -> p.toPose2d());

        if (tagPoseOpt.isEmpty()) return;

        Pose2d tagPose = tagPoseOpt.get();

        double dx = tagPose.getX() - robotPose.getX();
        double dy = tagPose.getY() - robotPose.getY();

        Rotation2d targetAngle =
            new Rotation2d(Math.atan2(dy, dx));

        drivetrain.setControl(
            request.withVelocityX(xSupplier.getAsDouble())
                   .withVelocityY(ySupplier.getAsDouble())
                   .withTargetDirection(targetAngle)
        );
    }

    private int getHubTagID() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()
            && alliance.get() == DriverStation.Alliance.Red) {
            return 10;
        }
        return 26;
    }
}