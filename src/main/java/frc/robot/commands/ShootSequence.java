import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMath;

package frc.robot.commands;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootSequence extends Command {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Kicker kicker;
    private final Hopper hopper;
    private final Vision vision;
    private final CommandSwerveDrivetrain swerve;

    public ShootSequence(Shooter shooter, Indexer indexer, Kicker kicker,
                         Hopper hopper, Vision vision,
                         CommandSwerveDrivetrain swerve) {

        this.shooter = shooter;
        this.indexer = indexer;
        this.kicker = kicker;
        this.hopper = hopper;
        this.vision = vision;
        this.swerve = swerve;

        addRequirements(shooter, indexer, kicker, hopper);
    }

    @Override
    public void initialize() {
        shooter.resetRamp();
        kicker.resetRamp();
    }

    @Override
    public void execute() {
        // 🔹 Compute RPMs EXACTLY like your commands do
        double shooterRPM = ShooterMath.getBestShooterRPM(swerve, vision);
        double kickerRPM  = ShooterMath.getBestKickerRPM(swerve, vision);

        // 🔹 Spin up both
        shooter.setRPM(shooterRPM);
        kicker.setRPM(kickerRPM);

        // 🔹 Feed ONLY when ready
        if (shooter.atSetPoint() && kicker.atSetPoint()) {
            indexer.setRPM(-4000);
            hopper.setRPM(3000);
        } else {
            indexer.stop();
            hopper.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        kicker.stop();
        indexer.stop();
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}