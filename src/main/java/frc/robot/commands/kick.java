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
      double angle = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      double rpm = ShooterMath.calcMotorRPM(ShooterMath.calcVelocity(distance, angle));
      rpm = rpm *(5/20); // account for gear ratio 
      kicker.setRPM(rpm);
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
