package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeArmFreeMoveCommand extends Command {
    private final Intake intake;
    private CommandXboxController manipulator;
    
    private double holdPosition;
    private boolean isHolding = false;

    // Adjust these constants based on testing
    private final double UP_MULTIPLIER = 0.4;   // Faster up
    private final double DOWN_MULTIPLIER = 0.1; // Slower down
    private final double kG = 0.05;             // Constant percent output to fight gravity
    private final double STICK_DEADBAND = 0.05;

    public IntakeArmFreeMoveCommand(Intake intake, CommandXboxController manipulator) {
        this.intake = intake;
        this.manipulator = manipulator;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Start by holding the current position
        holdPosition = intake.getArmPosition();
        isHolding = true;
    }

    @Override 
    public void execute() {
        double stickValue = -manipulator.getLeftY(); // Xbox Y is usually inverted
        
        if (Math.abs(stickValue) > STICK_DEADBAND) {
            // --- MANUAL MOVE MODE ---
            isHolding = false;
            
            double speed;
            if (stickValue > 0) {
                speed = stickValue * UP_MULTIPLIER;
            } else {
                speed = stickValue * DOWN_MULTIPLIER;
            }

            // Apply speed + gravity offset
            intake.setArmOutput(speed + kG);
            
        } else {
            // --- HOLD MODE ---
            if (!isHolding) {
                // Just released the stick? Lock in the current position
                holdPosition = intake.getArmPosition();
                isHolding = true;
            }
            
            // Simple P-loop to hold position: (Target - Current) * Sensitivity
            double error = holdPosition - intake.getArmPosition();
            double feedback = error * 0.1; // Adjust this 'P' gain until it holds firm
            
            intake.setArmOutput(feedback + kG);
        }

        SmartDashboard.putNumber("Arm Hold Target", holdPosition);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopArm();
    }
}