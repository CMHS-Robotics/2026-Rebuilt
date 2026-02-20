package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Kick extends Command{
    private final Kicker kicker;

    public Kick(Kicker kicker) {
        this.kicker = kicker;
        addRequirements(kicker); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        kicker.resetRamp();
    }

    @Override
    public void execute() {
      double distance = SmartDashboard.getNumber("Target Distance (m)", 0.0);
    //  double angleDeg =
    //  SmartDashboard.getNumber("Angle of Ejection (
    
    
    //  double velocity = ShooterMath.calcVelocity(distance, angleRad);
    //  double rpm = ShooterMath.calcMotorRPM(velocity);
    double rpm = ShooterMath.getRPM(distance);
    //double rpm = SmartDashboard.getNumber("SetRPM", 0);
      //double angleRad = Math.toRadians(angleDeg);
      //double velocity = ShooterMath.calcVelocity(distance, angleRad);
      //double rpm = ShooterMath.calcMotorRPM(velocity);
      //targetRPS /= (16/20);

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
