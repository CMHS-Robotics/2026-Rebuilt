package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Command;

public class runIntake extends Command{

    private final Intake intake;

    public runIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        intake.resetRamp();
    }

    @Override
    public void execute() {
      intake.startIntake(); //runs at 1000 rpm
    }

    @Override
    public void end(boolean interrupted) {
      intake.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false; // keep spinning until another command interrupts it
    }

}

