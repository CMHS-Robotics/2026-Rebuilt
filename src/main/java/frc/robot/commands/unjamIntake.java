package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;

public class unjamIntake extends Command {
    Hopper hopper;
    Indexer indexer;
    public unjamIntake(Hopper hopper, Indexer indexer){
        this.hopper = hopper;
        this.indexer = indexer;
        addRequirements(hopper,indexer);
    }

    @Override
    public void execute() {
    }
}
