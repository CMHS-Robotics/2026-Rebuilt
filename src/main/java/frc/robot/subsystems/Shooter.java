package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1 = new TalonFX(13);/////IMPORTANT
    private final TalonFX shooterMotor2 = new TalonFX(14);
    //IMPORTANT
    /*
     * 
     * DEVICE ID CHANGED TO DEBUG SWERVE CODE
     * 
     */
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
}
}