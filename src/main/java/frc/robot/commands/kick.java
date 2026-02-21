package frc.robot.commands;
import frc.robot.subsystems.*;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Kick extends Command{
    private final Kicker kicker;
    private final Vision vision;

    public Kick(Kicker kicker, Vision vision) {
        this.kicker = kicker;
        this.vision = vision;
        addRequirements(kicker); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        kicker.resetRamp();
    }

    @Override
    public void execute() {
    Optional<Double> primaryDistance   = vision.distanceToTagFromPose(10);
        Optional<Double> secondaryDistance = vision.distanceToTagFromPose(9);

        
        double distance;

        if (primaryDistance.isPresent()) {
            distance = vision.distanceToTagFromPose(10).orElse(Double.NaN);

        } else if (secondaryDistance.isPresent()) {
            distance = vision.distanceToTagFromPose(9).orElse(Double.NaN);

        } else {

            return;
        }

       double rpm = ShooterMath.getRPM(distance);
      kicker.setRPM(-rpm);
    }

    @Override
    public void end(boolean interrupted) {
      kicker.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }

}
