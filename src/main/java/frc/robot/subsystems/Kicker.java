package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Kicker extends SubsystemBase {
    private final TalonFX kickerMoter = new TalonFX(17); // Assuming CAN ID 4 for kicker motor
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(2000); // Limit to 2000 RPM per second
    private final double RPM_TOLLERANCE = 300;
    private double targetRPM = 0;

    public Kicker() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.12;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        config.Slot0.kS = 0.25;
        //config.Slot0.kF = 0.05;
       config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 100;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        kickerMoter.getConfigurator().apply(config);
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public void setRPM(double rpm) {

        this.targetRPM = rpm; // Store the target
        double rampedRPM = rpmRamp.calculate(rpm);
        double targetRPS = rampedRPM / 60.0;
        kickerMoter.setControl(velocityRequest.withVelocity(targetRPS));
        
    }

    public void stop() {
        kickerMoter.set(0);
    }



    public Command stopCommand() {
        return runOnce(this::stop);
    }

     @Override
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS1 = kickerMoter.getVelocity().getValueAsDouble();
    double currentRPM1 = currentRPS1 * 60.0;
    SmartDashboard.putNumber("KickerMoter RPM", currentRPM1);
}

    public boolean atSetPoint(){
        
        double currentRPM = kickerMoter.getVelocity().getValueAsDouble() * 60.0;
        
        // Use Math.abs to check if the difference is within the tolerance range
        return Math.abs(currentRPM - targetRPM) <= RPM_TOLLERANCE;
    }
}