package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper; // if you have one for feeding balls
import frc.robot.subsystems.Vision; // for getting distance and rot 
import frc.robot.subsystems.ShooterMath;
import edu.wpi.first.math.geometry.Rotation2d;  // for rotation calculations
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional; 

public class PointAndShoot extends Command {

    private final Shooter shooter;
    private final Hopper hopper;
    private final Vision vision;

    private final double toleranceRPM = 50.0;// flywheel RPM tolerance for ready
    private final double rotTollerace = Math.toRadians(5); // radians from degrees tolerance for aiming

    public PointAndShoot(Shooter shooter, Hopper hopper, Vision vision) {
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

        Rotation2d rotError;
        Optional<Rotation2d> rotTo10 = vision.getRotationErrorToTag(10);
        Optional<Rotation2d> rotTo26 = vision.getRotationErrorToTag(26);

        if(SmartDashboard.getString("Aliance", "Red").equals("Red")){
            if(!(rotTo10.isEmpty())){
                rotError = rotTo10.get();
            }
        } 
        else if(!(rotTo26.isEmpty())){
                rotError = rotTo26.get();
            }

        
        }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(0.0);
 //       hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // could be a timed shoot, or end via button release
    }

    
}