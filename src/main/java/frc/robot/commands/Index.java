package frc.robot.commands;
import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj2.command.Command;

public class Index extends Command{

    private final Indexer indexer;
    private final Kicker kicker;
    private final Shooter shooter;

    public Index(Indexer indexer, Shooter shooter, Kicker kicker) {
        this.indexer = indexer;
        this.kicker = kicker;
        this.shooter = shooter;
        addRequirements(indexer); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        indexer.resetRamp();
    }

    @Override
    public void execute() {
        // Only spin the indexer if shooter and kicker are ready
        if (shooter.atSetPoint() && kicker.atSetPoint()) {
            indexer.setRPM(2000); 
        } else {
            indexer.setRPM(0); // Stay still until up to speed
        }
    }

    @Override
    public void end(boolean interrupted) {
      indexer.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }

}

