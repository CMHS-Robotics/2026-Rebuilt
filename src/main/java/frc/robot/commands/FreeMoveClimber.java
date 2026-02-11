package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class FreeMoveClimber extends Command {
    private final Climber climber;
    public final Boolean up;
    public final Boolean down;

    public FreeMoveClimber(Climber climber, Boolean u, Boolean d) {
        this.climber = climber;
        this.up = u;
        this.down = d;
        addRequirements(climber);
    }

    @Override

    public void execute(){
        if(up == true && down == false){
            climber.moveUp();
        }else if(up == false && down == true){
            climber.moveDown();
        }else {
            climber.stopClimber();
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
