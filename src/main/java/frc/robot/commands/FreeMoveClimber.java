package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.FreeMoveStates;

public class FreeMoveClimber extends Command {
    private final Climber climber;
    public FreeMoveStates StateRequest;
    public FreeMoveStates CurrentState;

    public FreeMoveClimber(Climber climber, FreeMoveStates r) {
        this.climber = climber;
        this.StateRequest = r;

        addRequirements(climber);
    }

    @Override

    public void execute(){
        CurrentState = climber.currentFreeMoveState;

        switch(StateRequest) {
            case Enabled:
            CurrentState = FreeMoveStates.Enabled;
            climber.stopClimber();
            break;
            case Disabled:
                CurrentState = FreeMoveStates.Disabled;
                climber.stopClimber();
            break;
            case PositiveDirection:
                if(CurrentState!=FreeMoveStates.Disabled){//Prevents Climber From Moving If Current State Is Disabled
                    CurrentState = FreeMoveStates.PositiveDirection;
                    climber.moveUp();
                }
            break;
            case NegativeDirection:
                if(CurrentState!=FreeMoveStates.Disabled){//Prevents Climber From Moving If Current State Is Disabled
                    CurrentState = FreeMoveStates.NegativeDirection;
                    climber.moveDown();
                }
            break;

            default:
                CurrentState = FreeMoveStates.Disabled;//Defaults To Disabled State
            break;
        }
    }
    @Override
    public boolean isFinished() {
        return climber.atPosition(climber.getPosition());
    }

    @Override
    public void end(boolean interrupted){
        climber.stopClimber();
    }
}
