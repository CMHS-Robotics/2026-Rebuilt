package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmFeedPose extends Command {
    double positionDouble;
    Intake intake;
    public ArmFeedPose(Intake intake){
        this.positionDouble = positionDouble;
        this.intake = intake;
    }

    @Override
    public void execute() {
        if(Math.abs(intake.getArmPosition() - 11.5) >= 0.1){
            intake.setArmOutput(0.5);
        }else{
            intake.stopArm();
        }
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
