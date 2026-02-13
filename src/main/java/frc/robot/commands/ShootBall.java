package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterMath;

public class ShootBall extends Command {

    private final Shooter shooter;

    public ShootBall(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {
      //double distance =
      //SmartDashboard.getNumber("Target Distance (m)", 0.0);
      //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      //double angleRad = Math.toRadians(angleDeg);
      //double velocity = ShooterMath.calcVelocity(distance, angleRad);
      //double rpm = ShooterMath.calcMotorRPM(velocity);
      double rpm = SmartDashboard.getNumber("SetRPM", 0);
      double angle = SmartDashboard.getNumber("SetDegrees", 0);
      shooter.setRPM(rpm);
      shooter.setDegrees(angle);
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }
}