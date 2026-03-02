package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;

public class unjamRobot extends Command {
    Hopper hopper;
    Indexer indexer;
    public unjamRobot(Hopper hopper, Indexer indexer){
        this.hopper = hopper;
        this.indexer = indexer;
        addRequirements(hopper,indexer);
    }

    @Override
    public void execute() {
        indexer.setRPM(-2500);
        hopper.setRPM(-1000);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted){
        indexer.stop();
        hopper.stop();
    }
}
