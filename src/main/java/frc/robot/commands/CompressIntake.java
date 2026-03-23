package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class CompressIntake extends Command {
    Intake intake;
    public CompressIntake(Intake intake){
        this.intake = intake;
        //addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.compress();
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
