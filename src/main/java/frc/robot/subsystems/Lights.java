package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANdle;

public class Lights extends SubsystemBase {
    
    private final CANdle candle = new CANdle(0); // CAN ID

    public Lights() {
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
        
    }


}
