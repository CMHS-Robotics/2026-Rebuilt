package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class LockOnHub extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final Vision vision;

    private double lastValidErrorRad = 0;
    private int lostFramesCounter = 0;
    private final int MAX_LOST_FRAMES = 10;

   public LockOnHub(CommandSwerveDrivetrain drivetrain, Vision vision, 
                     DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.vision = vision;
        addRequirements(drivetrain);
    }

   @Override
    public void execute() {
        int tagID = getHubTagID();
        var errorOpt = vision.getRawRotationErrorToHubAnyTag();

        double omega;
        boolean isVisionValid = errorOpt.isPresent();

        if (isVisionValid) {
            // Update our "Ghost" data
            lostFramesCounter = 0;
            lastValidErrorRad = errorOpt.get().getRadians();
            omega = 5.0 * lastValidErrorRad;
        } else {
            lostFramesCounter++;
            if (lostFramesCounter <= MAX_LOST_FRAMES) {
                // Coast using the last known direction
                omega = 5.0 * lastValidErrorRad;
            } else {
                // Truly lost: Stop rotating, but keep driving!
                omega = 0;
            }
        }

        // Apply control: Keep X and Y active even if vision is dead
        SwerveRequest request = new SwerveRequest.FieldCentric()
            .withVelocityX(xSupplier.getAsDouble() * .6)
            .withVelocityY(ySupplier.getAsDouble() *.6)
            .withRotationalRate(omega);

        drivetrain.setControl(request);

        // Telemetry
        SmartDashboard.putBoolean("Lock Seen", isVisionValid);
        SmartDashboard.putNumber("Vision Lost Counter", lostFramesCounter);
        SmartDashboard.putNumber("Turn Power", omega);
    }

    private int getHubTagID() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
               == DriverStation.Alliance.Red ? 10 : 26;
    }
}