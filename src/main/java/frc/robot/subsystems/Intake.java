package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase{
    private final TalonFX intakeMoter = new TalonFX(19); 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(2500); // Limit to 500 RPM per second

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        //config.Slot0.kF = 0.05;

        intakeMoter.getConfigurator().apply(config);
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public void startIntake() {
        double rampedRPM = rpmRamp.calculate(5000);
        double targetRPS = rampedRPM / 60.0;
        intakeMoter.setControl(velocityRequest.withVelocity(-targetRPS));
    }

    public void stop() {
        intakeMoter.set(0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS1 = intakeMoter.getVelocity().getValueAsDouble();
    double currentRPM1 = currentRPS1 * 60.0;

    SmartDashboard.putNumber("intakeMoter RPM", currentRPM1);
    SmartDashboard.putNumber("intake compliant speed", currentRPM1 * (16.0/30.0)); // account for gear ratio
}
}
