package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.FreeMoveStates;


public class MoveClimber extends Command {
    private final Climber climber;
    private final double target;

    public MoveClimber(Climber climber, double target) {
        this.climber = climber;
        this.target = target;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        climber.currentFreeMoveState = FreeMoveStates.Disabled;
        climber.moveToward(target);
    }
    @Override
    public boolean isFinished() {
        return climber.atPosition(target);
    }

    @Override
    public void end(boolean interrupted){
        climber.stopClimber();
    }
}
