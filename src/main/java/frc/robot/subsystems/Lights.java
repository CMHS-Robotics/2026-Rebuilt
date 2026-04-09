package frc.robot.subsystems;

import java.util.EnumMap;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    private final CANdle m_CANdle;
    public Color m_Color;
    public int maxPos;
    public int minPos;
    public final boolean defaultColor = true;


    private final RainbowAnimation m_slot0Animation = new RainbowAnimation(29, 179)
        .withSlot(0)
        .withBrightness(1)
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(49.294));
    private final LarsonAnimation m_slot1Animation = new LarsonAnimation(0, 0)
        .withSlot(0)
        .withColor(new RGBWColor(255, 0, 0, 0))
        .withSize(10)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(50.274));
    private final LarsonAnimation m_slot2Animation = new LarsonAnimation(0, 0)
        .withSlot(0)
        .withColor(new RGBWColor(0, 0, 255, 0))
        .withSize(10)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(50.274));

    private final SolidColor[] m_colors = new SolidColor[] {
    };

    enum Color {
        Green,
        Red,
        Blue
    }

    public double[] RBGVALUE = {
        255,
        255,
        255
    };

    public Lights() {
        //Led Positions
        maxPos = 255;
        minPos = 0;

        m_CANdle = new CANdle(23);
        //Config
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 1;
        config.LED.StripType = StripTypeValue.GRB;
        m_CANdle.getConfigurator().apply(config);
        //Values
        m_Color = Color.Red;

        //SmartDashboard
        SmartDashboard.putString("Selected R/G/B Value",m_Color.toString());
        SmartDashboard.putNumberArray("RGB VALUE", RBGVALUE);

        setDefaultCommand(rainbowLeds());
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putString("Selected R/G/B Value",m_Color.toString());
        SmartDashboard.putNumberArray("RGB VALUE", RBGVALUE);
    }
    public void selectNextColor(){
        switch (m_Color) {
            case Red:
                m_Color = Color.Green;
                break;
            case Green:
                m_Color = Color.Blue;
                break;
            case Blue:
                m_Color = Color.Red;
                break;
            default:
                m_Color = Color.Red;
                break;
        }
    }
    public void selectPrevColor(){
        switch (m_Color) {
            case Red:
                m_Color = Color.Blue;
                break;
            case Green:
                m_Color = Color.Red;
                break;
            case Blue:
                m_Color = Color.Green;
                break;
            default:
                m_Color = Color.Red;
                break;
        }
    }
    public void incrementCurrent(){
        switch (m_Color) {
            case Red:
                RBGVALUE[0]++;
                RBGVALUE[0] = (RBGVALUE[0] > 255) ? 255 : RBGVALUE[0];
                break;
            case Green:
                RBGVALUE[1]++;
                RBGVALUE[1] = (RBGVALUE[1] > 255) ? 255 : RBGVALUE[1];
                break;
            case Blue:
                RBGVALUE[2]++;
                RBGVALUE[2] = (RBGVALUE[2] > 255) ? 255 : RBGVALUE[2];
                break;
            default:
                m_Color = Color.Red;
                RBGVALUE[0]++;
                RBGVALUE[0] = (RBGVALUE[0] > 255) ? 255 : RBGVALUE[0];
                break;
        }
    }
    public void decrementCurrent(){
        switch (m_Color) {
            case Red:
                RBGVALUE[0]--;
                RBGVALUE[0] = (RBGVALUE[0] < 0) ? 0 : RBGVALUE[0];
                break;
            case Green:
                RBGVALUE[1]--;
                RBGVALUE[1] = (RBGVALUE[1] < 0) ? 0 : RBGVALUE[1];
                break;
            case Blue:
                RBGVALUE[2]--;
                RBGVALUE[2] = (RBGVALUE[2] < 0) ? 0 : RBGVALUE[2];
                break;
            default:
                m_Color = Color.Red;
                RBGVALUE[0]--;
                RBGVALUE[0] = (RBGVALUE[0] < 0) ? 0 : RBGVALUE[0];
                break;
        }
    }
    public void UpdateColor(){
        
    }

    public Command rainbowLeds() {
        return run(() -> {
            for (var solidColor : m_colors) {
                m_CANdle.setControl(solidColor);
            }
            m_CANdle.setControl(m_slot0Animation);
        });
    }

    public Command redAuto() {
        return run(() -> {
            for (var solidColor : m_colors) {
                m_CANdle.setControl(solidColor);
            }
            m_CANdle.setControl(m_slot1Animation);
        });
    }
    public Command blueAuto() {
        return run(() -> {
            for (var solidColor : m_colors) {
                m_CANdle.setControl(solidColor);
            }
            m_CANdle.setControl(m_slot2Animation);
        });
    }
    public Command reset(){
        return run(()->{
            EmptyAnimation empty = new EmptyAnimation(0);
            m_CANdle.setControl(empty);
            m_CANdle.clearAllAnimations();
        });
    }
    @Override
    public Command idle() {
        return run(()->{
            rainbowLeds();
        });
    }
}
