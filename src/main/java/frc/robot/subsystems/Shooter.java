package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {

    private static final double HOOD_GEAR_RATIO = 9.0;
    private final TalonFX shooterMotor1 = new TalonFX(13);
    private final TalonFX shooterMotor2 = new TalonFX(14);
    private final TalonFX hoodMotor = new TalonFX(18);

    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);



    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(1500.0);

    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.12;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        shooterMotor1.getConfigurator().apply(config);
        shooterMotor2.getConfigurator().apply(config);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 10.0; // Start small!
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.1;

        // 1. Limit current so we don't melt the motor when it stalls at the bottom
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0; // 20 Amps is safe for stalling briefly
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // 2. Set the "Ceiling" (Top limit) so it never goes too far up
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodConfig);
    }

    public void setRPM(double rpm) {
        double rampedRPM = rpmRamp.calculate(rpm);
        double targetRPS = rampedRPM / 60.0;
        shooterMotor1.setControl(velocityRequest.withVelocity(targetRPS));
        shooterMotor2.setControl(velocityRequest.withVelocity(targetRPS));
    }

    public void stop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public void setHoodPosition(double angle) {
        double rotations = (angle / 360.0) * HOOD_GEAR_RATIO;
        hoodMotor.setControl(hoodPositionRequest.withPosition(rotations));
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
      }

    @Override
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS1 = shooterMotor1.getVelocity().getValueAsDouble();
    double currentRPM1 = currentRPS1 * 60.0;

    
    double currentRPS2 = shooterMotor1.getVelocity().getValueAsDouble();
    double currentRPM2 = currentRPS2 * 60.0;

    SmartDashboard.putNumber("ShooterMoter1 RPM", currentRPM1);
    SmartDashboard.putNumber("ShooterMoter2 RPM", currentRPM2);
    SmartDashboard.putNumber("Flywheel speed", currentRPM1 * (15.0/36.0)); // account for gear ratio
}
}