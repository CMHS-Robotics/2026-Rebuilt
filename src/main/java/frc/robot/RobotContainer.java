package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ShootBall;
//import frc.robot.apriltag.AprilTagFieldLayout;

public class RobotContainer {

  /* ================= SUBSYSTEMS ================= */
  private final Shooter shooter = new Shooter();
  
  /* ================= CONTROLLERS ================= */
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot */
  public RobotContainer() {

    // --- SmartDashboard tuning values ---
    SmartDashboard.putNumber("Target Distance (m)", 3.0);
    SmartDashboard.putNumber("Angle of Ejection (deg)", 60.0);


    //SmartDashboard.putData("Field View", vision.getFieldVisualizer()); 
    //add field view to dashboard once have vision subsystem

    configureBindings();
  }

  /* ================= BUTTON BINDINGS ================= */
  private void configureBindings() {

    // Hold RIGHT TRIGGER to spin shooter using dashboard distance
    driverController.rightTrigger()
        .whileTrue(new ShootBall(shooter));

    // Optional: stop shooter immediately on B
    driverController.b()
        .onTrue(shooter.stopCommand());
  }

  /* ================= AUTONOMOUS ================= */
  public Command getAutonomousCommand() {

    // Spin shooter for 2.5 seconds in auto (example)
    return new ShootBall(shooter).withTimeout(2.5);
  }
}