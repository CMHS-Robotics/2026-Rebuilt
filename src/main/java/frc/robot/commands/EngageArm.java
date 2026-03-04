package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class EngageArm extends Command {
    Intake intake;
    public EngageArm(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.engage();
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopArm();
    }

}
