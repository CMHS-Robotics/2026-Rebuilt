package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class FreeMoveClimber extends Command {
    private final Climber climber;
    public final BooleanSupplier up;
    public final BooleanSupplier down;

    public FreeMoveClimber(Climber climber, BooleanSupplier u, BooleanSupplier d) {
        this.climber = climber;
        this.up = u;
        this.down = d;
        addRequirements(climber);
    }

    @Override

    public void execute(){
        if(up.getAsBoolean() == true && down.getAsBoolean() == false){
            climber.moveUp();
        }else if(up.getAsBoolean() == false && down.getAsBoolean() == true){
            climber.moveDown();
        }else {
            climber.stopClimber();
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        climber.stopClimber();
    }
}
