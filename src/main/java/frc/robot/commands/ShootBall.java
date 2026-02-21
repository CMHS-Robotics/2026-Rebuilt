package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.tools.CalcFromVision;

public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision vision;
    private final CalcFromVision calc;


    public ShootBall(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.calc = new CalcFromVision(vision);
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {
      //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      //double angleRad = Math.toRadians(angleDeg);

      // double rpm = ShooterMath.getRPM(distance);
      double rpm = SmartDashboard.getNumber("SetRPM", 0);
     // double angle = SmartDashboard.getNumber("SetDegrees", 0);

  //   calc.calcHubRPM().ifPresent(rpm -> {
  //      SmartDashboard.putNumber("Calculated RPM", rpm);
  //      shooter.setRPM(rpm);
  //  });
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