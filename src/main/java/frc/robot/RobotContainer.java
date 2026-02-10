package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.FreeMoveClimber;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.ShootBall;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.MoveClimber;
import frc.robot.subsystems.Climber.*;

import frc.robot.generated.TunerConstants;

//import frc.robot.apriltag.AprilTagFieldLayout;

public class RobotContainer {

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Driver = new CommandXboxController(0);

    private final CommandXboxController Manipulator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    Climber climber = new Climber();



  /* ================= SUBSYSTEMS ================= */
  private final Shooter shooter = new Shooter();
  
  /* ================= CONTROLLERS ================= */
  //private final CommandXboxController Driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot */
  public RobotContainer() {

    // --- SmartDashboard tuning values ---
    SmartDashboard.putNumber("Target Distance (m)", 3.0);
    SmartDashboard.putNumber("Angle of Ejection (deg)", 68);
    SmartDashboard.putString("Free Move Climber State", climber.FreeMoveStateInterpreter(FreeMoveStates.Disabled));
    SmartDashboard.putNumber("Climber Position", climber.getPosition());
    SmartDashboard.putNumber("Stage", climber.stages[0]);


    ///////IMPLEMENT THE ABOVE SET FUNCTIONS IN CLASSES


   // SmartDashboard.putNumber("rpm", 0);


    //SmartDashboard.putData("Field View", vision.getFieldVisualizer()); 
    //add field view to dashboard once have vision subsystem

    configureBindings();
  }

  /* ================= BUTTON BINDINGS ================= */
  // private void configureBindings() {

  //   // Hold RIGHT TRIGGER to spin shooter using dashboard distance
  //   Driver.rightTrigger()
  //       .whileTrue(new ShootBall(shooter));

  //   // Optional: stop shooter immediately on B
  //   Driver.b()
  //       .onTrue(shooter.stopCommand());
  // }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        Driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //implement commands

        //Climber Commands:
        Manipulator.povUp().onTrue(
            new MoveClimber(climber,climber.stages[1])//Ground To Bar
        );
        Manipulator.povRight().onTrue(
            new MoveClimber(climber,climber.stages[0])//Bar To Bar
        );
        Manipulator.povDown().onTrue(
            new MoveClimber(climber,climber.stages[0])//Move To Bottom
        );
        //Free Move Commands For Climber
        Manipulator.povLeft().onTrue(
            new FreeMoveClimber(climber, FreeMoveStates.Enabled)
        );
        Manipulator.povDownLeft().onFalse(
            new FreeMoveClimber(climber,FreeMoveStates.Disabled)
        );
        Manipulator.rightTrigger().onTrue(
            new FreeMoveClimber(climber, FreeMoveStates.PositiveDirection)
        );
        Manipulator.rightTrigger().onFalse(
            new FreeMoveClimber(climber, FreeMoveStates.Enabled)
        );
        Manipulator.leftTrigger().onTrue(
            new FreeMoveClimber(climber, FreeMoveStates.NegativeDirection)
        );
        Manipulator.leftTrigger().onFalse(
            new FreeMoveClimber(climber, FreeMoveStates.Enabled)
        );

        //Change Above to Accept Trigger Pressed As A Value Between 0 and 1(or a set threshold), Invert For Left Side
        //Replace Current Toggle System With A Better One(Maybe?)
        //We Might Not Keep Free Mode Command, Maybe Only For Debug Purposes


        
        drivetrain.registerTelemetry(logger::telemeterize);
    }


  // /* ================= AUTONOMOUS ================= */
  // public Command getAutonomousCommand() {

  //   // Spin shooter for 2.5 seconds in auto (example)
  //   return new ShootBall(shooter).withTimeout(2.5);
  // }

  public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}