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

public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision vision;
    private final CommandSwerveDrivetrain swerve;
    private final FuelSim fs;
    private boolean hasFired = false;
    private final Timer m_timer = new Timer();
    private double m_lastShotTime = 0;
    private final double m_cooldown = 0.2; // 1.0 second / 5 balls = 0.2s



    public ShootBall(Shooter shooter, Vision vision, CommandSwerveDrivetrain swerve, FuelSim fs) {
        this.shooter = shooter;
        this.vision = vision;
        this.swerve = swerve;
        this.fs = fs;
        addRequirements(shooter); // This prevents other commands from using the shooter at the same time
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
        m_timer.restart(); // Start/Reset the timer when the command begins
        m_lastShotTime = -m_cooldown; // Allow immediate first shot
    }

    @Override
    public void execute() {
      //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
      //double angleRad = Math.toRadians(angleDeg);
      double rpm = SmartDashboard.getNumber("SetShooterRPM", 0);
     // double angle = SmartDashboard.getNumber("SetDegrees", 0);


    //double  rpm = ShooterMath.getBestShooterRPM(swerve, vision);
    

     shooter.setRPM(rpm);

     double currentTime = m_timer.get();

    // Condition 1: Is it time to shoot again? (Cooldown)
    // Condition 2: Is the shooter at the right speed?
    if ((currentTime - m_lastShotTime) >= m_cooldown) {
        
        double velocityMPS = (rpm * 2 * Math.PI * Units.inchesToMeters(4)) / 60.0;
        double exitVelocity = velocityMPS * 0.75; 

        fs.launchFuel(
            MetersPerSecond.of(exitVelocity),
            Degrees.of(70), 
            Degrees.of(0.0), 
            Meters.of(0.8)
        );

        m_lastShotTime = currentTime; // Update the timestamp of the last shot
    }

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