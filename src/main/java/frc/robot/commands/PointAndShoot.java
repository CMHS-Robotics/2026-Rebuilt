// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.IntakeSubsystem; // if you have one for feeding balls
// import frc.robot.subsystems.VisionSubsystem; // for getting distance

// public class PointAndShoot extends CommandBase {

//     private final Shooter shooter;
//     private final Intake intake;
//     private final Vision vision;

//     private final double toleranceRPM = 50.0; // flywheel RPM tolerance for “ready”

//     public PointAndShoot(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision) {
//         this.shooter = shooter;
//         this.intake = intake;
//         this.vision = vision;
//         addRequirements(shooter, intake); // ensures no conflicts
//     }

//     @Override
//     public void initialize() {
//         shooter.resetRamp();
//     }

//     @Override
//     public void execute() {
//         double distance = vision.distanceToTag(); //wrote an idea for this function 
//         double theta = 60;      // this needs logic to calculate gotta figure that out later
//         double targetRPM = ShooterMath.calcVelocity(distance, theta).calcRPM();
//         shooter.setRPM(targetRPM);

//         //feed ball when RPM is within tolerance
//         if (Math.abs(targetRPM - shooter.getCurrentRPM()) < toleranceRPM) {
//             intake.runFeeder(); // spin feeder motor
//         } else {
//             intake.stopFeeder();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooter.setRPM(0.0);
//         intake.stopFeeder();
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // could be a timed shoot, or end via button release
//     }

    
// }