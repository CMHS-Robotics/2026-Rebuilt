package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(13); // Assuming CAN ID 4 for kicker motor
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(2500); // Limit to 500 RPM per second

    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        //config.Slot0.kF = 0.05;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
         config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        indexerMotor.getConfigurator().apply(config);
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public void setRPM(double rpm) {
        double rampedRPM = rpmRamp.calculate(rpm);
        double targetRPS = rampedRPM / 60.0;
        indexerMotor.setControl(velocityRequest.withVelocity(targetRPS));
        
    }

    public void stop() {
        indexerMotor.set(0);
    }



    public Command stopCommand() {
        return runOnce(this::stop);
    }

     @Override
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS = indexerMotor.getVelocity().getValueAsDouble();
    double currentRPM = currentRPS * 60.0;

    SmartDashboard.putNumber("indexerRPM", currentRPM);
}
}

