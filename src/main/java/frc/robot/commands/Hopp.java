package frc.robot.commands;
import frc.robot.subsystems.*;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Hopp extends Command{

    private final Hopper hopper;
    private final Vision vision;

    public Hopp(Hopper hopper, Vision vision) {
        this.hopper = hopper;
        this.vision = vision;
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
         double rpm = SmartDashboard.getNumber("SetRPM", 0);
    //    calc.calcHubRPM().ifPresent(rpm -> {
    //    SmartDashboard.putNumber("Calculated RPM", rpm);
    //    hopper.setRPM(rpm);
    //    });
      hopper.setRPM(rpm);
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


