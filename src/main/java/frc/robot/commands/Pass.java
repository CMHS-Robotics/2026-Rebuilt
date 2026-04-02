package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.robot.tools.FuelSim;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pass extends Command{

    private final Shooter shooter;
    private final Vision vision;
    private final CommandSwerveDrivetrain swerve;
    private boolean hasFired = false;
    private final Timer m_timer = new Timer();
    private double m_lastShotTime = 0;
    private final double m_cooldown = 0.2; // 1.0 second / 5 balls = 0.2s



    public Pass(Shooter shooter, Vision vision, CommandSwerveDrivetrain swerve) {
        this.shooter = shooter;
        this.vision = vision;
        this.swerve = swerve;
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
    }

    @Override
    public void execute() {

    double  rpm = ShooterMath.getBestShooterRPM(swerve, vision);
    SmartDashboard.putNumber("rpmCalced", rpm);
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

