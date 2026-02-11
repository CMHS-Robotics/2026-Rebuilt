package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase{
    private final TalonFX intakeMoter = new TalonFX(19); 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(500); // Limit to 500 RPM per second

    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        //config.Slot0.kF = 0.05;

        intakeMoter.getConfigurator().apply(config);
    }

    public void startIntake() {
        double rampedRPM = rpmRamp.calculate(1000);
        double targetRPS = rampedRPM / 60.0;
        intakeMoter.setControl(velocityRequest.withVelocity(targetRPS));
    }

    public void stop() {
        intakeMoter.set(0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
