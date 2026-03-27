package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;

public class Kick extends Command{
    private final Kicker kicker;
    private final Vision vision;
    private final CommandSwerveDrivetrain swerve;

    public Kick(Kicker kicker, Vision vision, CommandSwerveDrivetrain swerve) {
        this.kicker = kicker;
        this.vision = vision;
        this.swerve = swerve;
        addRequirements(kicker); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        kicker.resetRamp();
    }

    @Override
    public void execute() {
         //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
         //double angleRad = Math.toRadians(angleDeg);
    
       //  double rpm = SmartDashboard.getNumber("SetKickerRPM", 0);

        double rpm = ShooterMath.getBestKickerRPM(swerve, vision);


    //    double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
    //   double rpm = ShooterMath.getRPM(distance);
        kicker.setRPM(rpm);
     // double distance = SmartDashboard.getNumber("Target Distance (m)", 3.0);
     //  double rpm = ShooterMath.getRPM(distance);
     //   kicker.setRPM(-rpm);
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
