package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterMath;
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

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

   @Override
    public void execute() {

    double distance = vision.getBestHubDistance();
    // 1. Get the error. This uses your fused Pose, so it's always "live" 
    // even if the camera is currently blocked.
    var errorOptional = vision.getRotationErrorShooterToTagFromPose();

    // 2. Safety Gate: If the tag doesn't exist in the layout, stop.
    if (errorOptional.isEmpty()) {
        drivetrain.setControl(zero);
        return;
    }

    // 3. Extract the rotation error
    double errorRad = errorOptional.get().getRadians();

    // 4. Deadband: Stop vibrating when we are "close enough"
    if (Math.abs(errorRad) < ShooterMath.getShotTolerance(distance)) {
        drivetrain.setControl(zero);
    } else {
        // 5. PID Calculation
        // Use a P-loop to turn faster when the error is large
        double turnPower = MathUtil.clamp(kP * errorRad, -4.0, 4.0);
        
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(turnPower));
    }
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