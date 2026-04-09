// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShootBall;
import frc.robot.generated.TunerConstantsOld;
import frc.robot.subsystems.*;
import frc.robot.tools.FuelSim;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.units.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Lights;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private FuelSim m_fuelSim;
  private Lights lights = new Lights();

  @Override
  public void robotInit() {
    lights.reset();
    lights.setDefaultCommand(lights.rainbowLeds());
    lights.rainbowLeds();
    

    Logger.recordMetadata("ProjectName", "Robot");

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.addDataReceiver(new NT4Publisher());
} else {
    Logger.addDataReceiver(new NT4Publisher());
}

if (isSimulation()) {
    setUseTiming(false);
}

Logger.start();
m_robotContainer = new RobotContainer();
m_fuelSim = m_robotContainer.fuelSim;
CommandSwerveDrivetrain m_drive = m_robotContainer.getDrivetrain();

// 1. Tell the sim about your robot's size and how to find it
    m_fuelSim.registerRobot(
        Units.inchesToMeters(27), // Width
        Units.inchesToMeters(27), // Length
        Units.inchesToMeters(6), // Bumper Height
        () -> m_drive.getPose(),  // Supplier for robot pose
        () -> m_drive.getFieldRelativeSpeeds() // Supplier for velocity
    );

    // 2. Spawn the game pieces
    //m_fuelSim.spawnStartingFuel();
    m_fuelSim.start();
    m_fuelSim.enableAirResistance();
    m_fuelSim.setLoggingFrequency(50);
    m_fuelSim.setSubticks(50);


//    m_robotContainer.getDrivetrain().resetPose(new Pose2d(
//    new Translation2d(0, 0),
//));


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //System.out.println( m_robotContainer.getDrivetrain().getPose());
  }

  @Override
public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
        // 1. Trust PathPlanner for the starting spot
        Pose2d startPose = m_robotContainer.getAutoStartingPose(); 
        
        if (startPose != null) {
    // Get the current rotation from the gyro so we don't 'snap' the heading
    Rotation2d currentRotation = m_robotContainer.getDrivetrain().getState().Pose.getRotation();
    
    // Create a new pose: PathPlanner's X/Y, but the Gyro's current Angle
    Pose2d poseWithGyroHeading = new Pose2d(startPose.getTranslation(), currentRotation);
    
    m_robotContainer.getDrivetrain().resetPose(poseWithGyroHeading);
}
        // 2. Disable fusion briefly to prevent "jumps" during the first move
        m_robotContainer.getVision().setVisionEnabled(false);
        
        m_autonomousCommand.schedule();

        // 3. Re-enable fusion after 1 second once the robot is moving/settled
        new WaitCommand(.75)
            .andThen(() -> m_robotContainer.getVision().setVisionEnabled(true))
            .ignoringDisable(true)
            .schedule();
    }
}

   // Pose2d startingPose = m_autonomousCommand.getStartingPose();
   //swerve.resetPose(startingPose);


  @Override
  public void autonomousExit() {
    if(m_autonomousCommand != null){
      CommandScheduler.getInstance().cancel();
    }
  }
  

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }

     m_robotContainer.getVision().setVisionEnabled(true);
    m_robotContainer.getVision().getEstimatedPose().ifPresent(pose -> {
    m_robotContainer.getDrivetrain().resetPose(pose); 
});
    if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }



  }

  @Override
  public void teleopPeriodic() {

    //ShootBall ballShooter  = new ShootBall(shooter); 

    //ballShooter.execute();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
   m_robotContainer.getVision().simulationPeriodic();
   m_fuelSim.updateSim();

 }
}
