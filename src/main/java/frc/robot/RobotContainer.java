package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.tools.FuelSim;
import frc.robot.commands.*;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  

  private final SendableChooser<Command> autoChooser;

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

    public final FuelSim fuelSim = new FuelSim();

    


    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Create a specific request for slow mode
private final SwerveRequest.FieldCentric slowDriveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(0.1).withRotationalDeadband(0.1); // Add deadbands


    
    /* ================= FIELD LAYOUT ================= */
    private final AprilTagFieldLayout layout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);



  /* ================= SUBSYSTEMS ================= */
  private final Vision vision = new Vision(drivetrain);
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter(vision);
  private final Kicker kicker = new Kicker();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Hopper hopper = new Hopper();
  /* ================= CONTROLLERS ================= */
  //private final CommandXboxController Driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot */
  public RobotContainer() {

    ShootBall shootBallCommand = new ShootBall(shooter, vision, drivetrain, fuelSim);
    runIntake runIntakeCommand = new runIntake(intake);
    PointAndRotate alignToHubCommand = new PointAndRotate(drivetrain, vision);
    Hopp hoppCommand = new Hopp(hopper, shooter, kicker);
    Kick kickCommand = new Kick(kicker, vision, drivetrain);
    Index indexCommand = new Index(indexer, shooter, kicker);
    EngageIntake EngageIntake = new EngageIntake(intake);
    CompressIntake CompressIntake = new CompressIntake(intake);
    ArmFeedPose armFeedPose = new ArmFeedPose( intake);

    NamedCommands.registerCommand("shoot", shootBallCommand
        .alongWith(new Index(indexer, shooter, kicker).withTimeout(20))
        .alongWith(kickCommand)
        .alongWith(hoppCommand)
        .alongWith(EngageIntake)
        .withTimeout(20));

    
    NamedCommands.registerCommand("compressIntake", CompressIntake);
    NamedCommands.registerCommand("engageIntake", EngageIntake);
    NamedCommands.registerCommand("intakeSpin", runIntakeCommand);
    NamedCommands.registerCommand("intakeSpinUp", runIntakeCommand);
    NamedCommands.registerCommand("armFeedAngle", armFeedPose);
    NamedCommands.registerCommand("alignToHubCommand", alignToHubCommand);


  

    configureAutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Default",new InstantCommand());
    SmartDashboard.updateValues();
    SmartDashboard.putData("Auto Chooser",autoChooser);
    // --- SmartDashboard tuning values ---
    SmartDashboard.putNumber("Target Distance (m)", 3.0);
    SmartDashboard.putNumber("Angle of Ejection (deg)", 68);
    SmartDashboard.putNumber("Climber Position", climber.getPosition());
    SmartDashboard.putNumber("Stage", climber.stages[0]);
    SmartDashboard.putNumber("SetKickerRPM",0);
    SmartDashboard.putNumber("SetShooterRPM",0);

    SmartDashboard.putNumber("SetDegrees", 0);


    //DEBUG
    SmartDashboard.putBoolean("DEBUG COMMAND EXECUTES",false);
    SmartDashboard.putNumber("DEBUG RAW LEFT STICK DOUBLE",0.0);
    SmartDashboard.putBoolean("DEBUG PASSES STICKDRIFT",false);
    SmartDashboard.putNumber("DEBUG MULTIPLIER APPLIED", 0.0);
    SmartDashboard.putNumber("DEBUG NEGATIVE SQRT VALUE", 0.0);
    SmartDashboard.putNumber("DEBUG POSITIVE SQRT VALUE", 0.0);

    
    
   // SmartDashboard.putNumber("Feild", fieldVisualizer);


    ///////IMPLEMENT THE ABOVE SET FUNCTIONS IN CLASSES


   // SmartDashboard.putNumber("rpm", 0);


    //SmartDashboard.putData("Field View", vision.getFieldVisualizer()); 
    //add field view to dashboard once have vision subsystem

    configureBindings();
  }


  /* ================= BUTTON BINDINGS ================= */


          


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left( according to WPILib convention.
       Manipulator.rightTrigger().whileTrue(new ShootBall(shooter, vision, drivetrain, fuelSim));
       Driver.button(1).whileTrue( new ShootBall(shooter, vision, drivetrain, fuelSim)); //bind to spacebar for sim
       Manipulator.rightTrigger().whileTrue(new Kick(kicker, vision, drivetrain));
       Manipulator.rightTrigger().whileTrue(new Index(indexer, shooter, kicker));

       Manipulator.rightBumper().whileTrue(new Index(indexer, shooter, kicker));
       Manipulator.rightBumper().whileTrue(new Hopp(hopper, shooter, kicker));
       Driver.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
       
        Manipulator.leftTrigger().whileTrue(new runIntake(intake));

        Driver.y().whileTrue(new PointAndRotate(drivetrain, vision));


        Driver.button(2).whileTrue(new PointAndRotate(drivetrain, vision)); //bind to spacebar for sim

        Manipulator.y().onTrue(new EngageIntake(intake));
            
      

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        Driver.x().whileTrue(
          new LockOnHub(
              drivetrain,
              vision,
              () -> -Driver.getLeftY() * MaxSpeed,
              () -> -Driver.getLeftX() * MaxSpeed
          )
      );
       Driver.button(3).whileTrue( new LockOnHub(
              drivetrain,
              vision,
              () -> -Driver.getLeftY() * MaxSpeed,
              () -> -Driver.getLeftX() * MaxSpeed
          )); //bind to spacebar for sim

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
            new MoveClimber(climber,climber.stages[2])//Bar To Bar
        );
        Manipulator.povDown().onTrue(
            new MoveClimber(climber,climber.stages[0])//Move To Bottom
        );
        Manipulator.povLeft().whileTrue(
            new FreeMoveClimber(climber,Manipulator.rightBumper(),Manipulator.leftBumper())
        );
        //freemove Intake Arm Command
        Manipulator.x().whileTrue( new unjamRobot(hopper, indexer));

        Trigger stickMoved = new Trigger( () -> Math.abs(Manipulator.getLeftY()) > 0.1);
        stickMoved.whileTrue(
          new IntakeArmFreeMoveCommand(intake, Manipulator)
        );
        drivetrain.registerTelemetry(logger::telemeterize);

      //  Manipulator.leftBumper().whileTrue();



    Driver.rightTrigger().whileTrue(
      drivetrain.applyRequest(() -> 
        slowDriveRequest.withVelocityX(-Driver.getLeftY() * 0.6) // 40% Speed
            .withVelocityY(-Driver.getLeftX() * 0.6) 
            .withRotationalRate(-Driver.getRightX() * 0.7)
    )
);
        

    }




    public void configureAutoBuilder() {
      RobotConfig config;
      try{
        config = RobotConfig.fromGUISettings();
     }catch(Exception e){
        throw new RuntimeException("Failed to load PathPlanner Robot Config");
      }

      AutoBuilder.configure(
        ()->drivetrain.getPose(),
        drivetrain::resetPose,
        ()->drivetrain.getRobotRelativeSpeeds(),
        (speeds,Feedforwards) ->drivetrain.setControl(autoRequest.withSpeeds(speeds)),
          new PPHolonomicDriveController(
            new PIDConstants(5.0,0,0),
            new PIDConstants(5.0,0,0)
          ),
          config, 
          () -> DriverStation.getAlliance()
          .map(a -> a == DriverStation.Alliance.Red).orElse(false),
          drivetrain
      );
    }


  // /* ================= AUTONOMOUS ================= */
  // public Command getAutonomousCommand() {

  //   // Spin shooter for 2.5 seconds in auto (example)
  //   return new ShootBall(shooter).withTimeout(2.5);
  // }

  //public Command getAutonomousCommand() {
        // Simple drive forward auto
  //      return kick(kicker, vision);
  //  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
}

public Vision getVision() {
    return vision;
}
}