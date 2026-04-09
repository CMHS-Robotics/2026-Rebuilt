package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class LightsControl extends Command {
    Lights lights = new Lights();
    FLAG m_flag;
    public LightsControl(FLAG m_flag) {
        this.m_flag = m_flag;
    }

    @Override
    public void execute() {
        lights.reset();
        lights.setDefaultCommand(lights.redAuto());
        lights.redAuto();
        // lights.reset();
        // switch (m_flag) {
        //     case ColorTypeN:
        //         lights.selectNextColor();
        //         break;
        //     case ColorTypeP:
        //         lights.selectPrevColor();
        //         break;
        //     case ColorValueI:
        //         lights.incrementCurrent();
        //         break;
        //     case ColorValueD:
        //         lights.decrementCurrent();
        //         break;
        //     case BrightnessI:
        //         break;
        //     case BrightnessD:
        //         break;
        //     case NULL:
        //         lights.reset();
        //     case Rainbow:
        //         lights.rainbowLeds();
        //     case Blue:
        //         lights.blueAuto();
        //     case Red: 
        //         lights.redAuto();
        //     default:
        //         m_flag = FLAG.NULL;
        //         break;
        // }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    public enum FLAG {
        ColorTypeN,
        ColorTypeP,
        ColorValueI,
        ColorValueD,
        BrightnessI,
        BrightnessD,
        Rainbow,
        Blue,
        Red,
        FlashBang,
        NULL
    }

}
