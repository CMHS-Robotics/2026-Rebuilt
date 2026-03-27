package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor = new TalonFX(10000);
    private final TalonFX intakeUpDownMotor = new TalonFX(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(3500); // Limit to 500 RPM per second
    private final PositionVoltage intakeEngangeRequest = new PositionVoltage(0);

    private final PositionVoltage intakeCompressRequest = new PositionVoltage(0).withVelocity(.2);

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        //config.Slot0.kF = 0.05;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        

        TalonFXConfiguration upDownConfig = new TalonFXConfiguration();
        upDownConfig.Slot0.kP = 10.0; // Start small!
        upDownConfig.Slot0.kI = 0.0;
        upDownConfig.Slot0.kD = 0.1;

        // 1. Limit current so we don't melt the motor when it stalls at the bottom
        upDownConfig.CurrentLimits.StatorCurrentLimit = 25.0; // 20 Amps is safe for stalling briefly
        upDownConfig.CurrentLimits.StatorCurrentLimitEnable = true;
         upDownConfig.CurrentLimits.SupplyCurrentLimit = 25;
        upDownConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 2. Set the "Ceiling" (Top limit) so it never goes too far up
       // upDownConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2; //tune this
       // upDownConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

     //   upDownConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1630859375; //tune this 
       // upDownConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        intakeUpDownMotor.getConfigurator().apply(upDownConfig);

      //  intakeMoter.getConfigurator().apply(config);
        intakeMotor.getConfigurator().apply(config);
    }

    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public void startIntake() {
        double rampedRPM = rpmRamp.calculate(6250);
        double targetRPS = rampedRPM / 60.0;
        intakeMotor.setControl(velocityRequest.withVelocity(-targetRPS));
    }

    public void engage(){
        double motorRotations = 1;
        
        //-0.97705078125
        intakeUpDownMotor.setControl(intakeEngangeRequest.withPosition(motorRotations));
    }

    public void compress(){
        double motorRotations = -1;
        
        //-0.97705078125
        intakeUpDownMotor.setControl(intakeCompressRequest.withPosition(motorRotations));
    }

    public void stop() {
        intakeMotor.set(0);
    }
    public double getArmPosition(){
        return intakeUpDownMotor.getPosition().getValueAsDouble();
    }
    public void setArmOutput(double MotorOutput){
        intakeUpDownMotor.set(MotorOutput);
    }
    public void stopArm(){
        intakeUpDownMotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)


    double currentRPS1 = intakeMotor.getVelocity().getValueAsDouble();
    double currentRPM1 = currentRPS1 * 60.0;

    SmartDashboard.putNumber("intakeMoter RPM", currentRPM1);
    SmartDashboard.putNumber("intake compliant speed", currentRPM1 * (16.0/30.0)); // account for gear ratio
    SmartDashboard.putNumber("intake position", getArmPosition());    
}
}
