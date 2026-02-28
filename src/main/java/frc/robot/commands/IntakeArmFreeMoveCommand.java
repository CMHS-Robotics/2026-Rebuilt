package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeArmFreeMoveCommand extends Command {
    private final Intake intake;
    private CommandXboxController Manipulator;
    private double stickAsDouble;
    private final Double multiplier;
    //private final double positionTolerence;
    public IntakeArmFreeMoveCommand(Intake intake, CommandXboxController Manipulator) {
        //positionTolerence = 0.5;
        this.intake = intake;
        this.Manipulator = Manipulator;
        multiplier = 0.25;
        addRequirements(intake);
    }

    public double getPosition(){
        return intake.getArmPosition();
    }
    // public boolean atPosition(double target){
    //     return Math.abs(getPosition() - target) <= positionTolerence;
    // }

    @Override
    public void initialize() {
        //intake.getArmPosition();
    }
    @Override 
    public void execute() {
        stickAsDouble = Manipulator.getLeftY();
        SmartDashboard.putBoolean("DEBUG COMMAND EXECUTES",true);
        SmartDashboard.putNumber("DEBUG RAW LEFT STICK DOUBLE",stickAsDouble);
        //Prevents SLIGHT Stick Drift From Affecting 
        if(stickAsDouble>=0.05 || stickAsDouble <= -0.05){
            SmartDashboard.putBoolean("DEBUG PASSES STICKDRIFT",true);
            //applies a multiplier for added precision
            stickAsDouble*=multiplier;
            SmartDashboard.putNumber("DEBUG MULTIPLIER APPLIED", stickAsDouble);
            intake.setArmOutput(stickAsDouble);
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
