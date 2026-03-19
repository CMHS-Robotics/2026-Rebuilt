//DEPRECATED
//KEEPING FILE JUST INCASE


package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SlowDrive extends Command {

    CommandSwerveDrivetrain drivetrain;
    Optional<Double> multiplier;
    SwerveRequest.FieldCentric drive;

    public SlowDrive(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric drive,Optional<Double> multiplier){
        this.drivetrain = drivetrain;
        this.multiplier = multiplier;
        this.drive = drive;
    }

    @Override
    public void execute() {
        drivetrain.applyRequest(
            ()->drive.withVelocityX(0.1)
            .withVelocityY(0.1)
            .withRotationalRate(0.1)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
