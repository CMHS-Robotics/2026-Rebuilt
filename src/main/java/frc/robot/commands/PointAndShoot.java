package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem; // if you have one for feeding balls
import frc.robot.subsystems.VisionSubsystem; // for getting distance and rot 
import frc.robot.util.ShooterMath;
import edu.wpi.first.math.geometry.Rotation2d;  // for rotation calculations

public class PointAndShoot extends CommandBase {

    private final Shooter shooter;
    private final Hopper hopper;
    private final Vision vision;

    private final double toleranceRPM = 50.0;// flywheel RPM tolerance for ready
    private final double rotTollerace = Math.toRadians(5); // radians from degrees tolerance for aiming

    public PointAndShoot(ShooterSubsystem shooter, Hopper hopper, VisionSubsystem vision) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.vision = vision;
        addRequirements(shooter, hopper); // ensures no conflicts
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {
    
        Optional<Double> distanceOpt = vision.distanceToTagFromPose(); // idk what hub tag is
        Optional<Rotation2d> rotErrorOpt =
            vision.getRotationErrorToTag(); // idk what hub tag is but plug in here and above
    
        if (distanceOpt.isEmpty() || rotErrorOpt.isEmpty()) {
            hopper.stopFeeder();
            return;
        }
    
        double distance = distanceOpt.get();
        double theta = 60; // still fine for now
    
        double targetRPM =
            ShooterMath.calcRPM(shooterMath.calcVelocity(distance, theta));
    
        shooter.setRPM(targetRPM);
    
        boolean rpmReady =
            Math.abs(targetRPM - shooter.getCurrentRPM()) < toleranceRPM;
    
        boolean aimed =
            Math.abs(rotErrorOpt.get().getRadians()) < rotTollerace;
    
        if (rpmReady && aimed) {
            hopper.runFeeder();
        } else {
            hopper.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(0.0);
        hopper.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false; // could be a timed shoot, or end via button release
    }

    
}