package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBall extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetRPM;

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
      double distance =
      SmartDashboard.getNumber("Target Distance (m)", 0.0);
      double angle =
      SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);

  double rpm = ShooterMath.calcVelocity(distance, angle).calcRPM();
  shooter.setRPM(rpm);
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