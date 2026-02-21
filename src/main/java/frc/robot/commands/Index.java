package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.tools.CalcFromVision;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Index extends Command{

    private final Indexer indexer;
    private final Vision vision;
    private final CalcFromVision calc;

    public Index(Indexer indexer, Vision vision) {
        this.indexer = indexer;
        this.vision = vision;
        this.calc = new CalcFromVision(vision);
        addRequirements(indexer); // This prevents other commands from using the kicker at the same time
    }

    @Override
    public void initialize() {
        indexer.resetRamp();
    }

    @Override
    
    public void execute() {

         //double angleDeg = SmartDashboard.getNumber("Angle of Ejection (deg)", 60.0);
         //double angleRad = Math.toRadians(angleDeg);
    
         // double rpm = ShooterMath.getRPM(distance);
         double rpm = SmartDashboard.getNumber("SetRPM", 0);
    //    calc.calcHubRPM().ifPresent(rpm -> {
    //    SmartDashboard.putNumber("Calculated RPM", rpm);
    //    indexer.setRPM(rpm);
    //    });
          indexer.setRPM(rpm);
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

