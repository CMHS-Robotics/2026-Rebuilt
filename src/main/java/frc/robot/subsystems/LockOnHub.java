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

    public LockOnHub(
        CommandSwerveDrivetrain drivetrain,
        Vision vision,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.vision = vision;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
    
        int tagID = getHubTagID();
    
        var errorOpt = vision.getDirectRotationErrorShooterToTag(tagID);
    
        if (errorOpt.isEmpty()) return;

    
        double omega = 5.0 *(errorOpt.get().getRadians()); // tune this
    
        SwerveRequest request = new SwerveRequest.FieldCentric() //try robot centric if this is cooked
         .withVelocityX(xSupplier.getAsDouble())
         .withVelocityY(ySupplier.getAsDouble())
         .withRotationalRate(omega);

         drivetrain.setControl(request);
         SmartDashboard.putNumber("Heading", drivetrain.getState().Pose.getRotation().getDegrees());
         SmartDashboard.putNumber("Error", errorOpt.get().getDegrees());
         SmartDashboard.putBoolean("Lock Seen", errorOpt.isPresent());
         SmartDashboard.putNumber("Rotation Error Deg", Math.toDegrees(errorOpt.get().getRadians()));
         SmartDashboard.putNumber("Turn Power", omega);
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