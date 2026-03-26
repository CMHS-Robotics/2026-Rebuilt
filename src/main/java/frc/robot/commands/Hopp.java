package frc.robot.commands;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

public class Hopp extends Command{

    private final Hopper hopper;
    private final Shooter shooter;
    private final Kicker kicker;


    public Hopp(Hopper hopper, Shooter shooter, Kicker kicker) {
        this.hopper = hopper;
        this.shooter = shooter;
        this.kicker = kicker;
        addRequirements(hopper); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        hopper.resetRamp();
    }

    @Override
    public void execute() {
         //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
         //double angleRad = Math.toRadians(angleDeg);
         // double rpm = ShooterMath.getRPM(distance);
    //     double rpm = SmartDashboard.getNumber("SetRPM", 0);
    //    calc.calcHubRPM().ifPresent(rpm -> {
    //    SmartDashboard.putNumber("Calculated RPM", rpm);
    //    hopper.setRPM(rpm);
    //    });

    //double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
       // Only spin the indexer if shooter and kicker are ready
        if (shooter.atSetPoint() && kicker.atSetPoint()) {
            hopper.setRPM(2000); 
        } else {
            hopper.setRPM(0); // Stay still until up to speed
       }
    }

    @Override
    public void end(boolean interrupted) {
      hopper.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }

}


