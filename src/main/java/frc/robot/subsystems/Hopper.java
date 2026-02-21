package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Hopper extends SubsystemBase {
     private final TalonFX hopperMoter = new TalonFX(16); 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(2000); // Limit to 500 RPM per second

    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        //config.Slot0.kF = 0.05;

        hopperMoter.getConfigurator().apply(config);
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public void setRPM(double rpm) {
        double rampedRPM = rpmRamp.calculate(rpm);
        double targetRPS = rampedRPM / 60.0;
        hopperMoter.setControl(velocityRequest.withVelocity(targetRPS)) ;
        
    }

    public void stop() {
        hopperMoter.set(0);
    }



    public Command stopCommand() {
        return runOnce(this::stop);
    }

     @Override
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS1 = hopperMoter.getVelocity().getValueAsDouble();
    double currentRPM1 = currentRPS1 * 60.0;
    SmartDashboard.putNumber("hopperMoter RPM", currentRPM1);
}
}