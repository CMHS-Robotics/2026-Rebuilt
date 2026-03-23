package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class EngageIntake extends Command {
    Intake intake;
    public EngageIntake(Intake intake){
        this.intake = intake;
        //addRequirements(intake);
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
