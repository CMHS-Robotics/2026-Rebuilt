package frc.robot.CustomUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.tools.PID;

public class C_Motor {

    TalonFX Motor;
    PID pid;
    TalonFXConfiguration config;

    public C_Motor(int id) {
        Motor = new TalonFX(id);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;
    }

    TalonFX initMotor(){
        Motor.getConfigurator().apply(config);
        Motor.setPosition(0);
        return this.Motor;
    }

    

}
