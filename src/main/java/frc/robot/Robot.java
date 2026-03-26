// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
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

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private FuelSim m_fuelSim;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "Robot");

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
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
      // 1. Get the starting pose from PathPlanner
      // Note: This requires you to have a way to know WHICH auto is selected.
      // If you are using a SendableChooser, you might need to get the name from there.
      Pose2d startPose = m_robotContainer.getAutoStartingPose(); 
      
      // 2. Reset the drivetrain to that pose before running the command
      if (startPose != null) {
          m_robotContainer.getDrivetrain().resetPose(startPose);
      }

      // 3. Schedule the auto command
      m_autonomousCommand.schedule();

      m_robotContainer.getVision().setVisionEnabled(false);


    CommandScheduler.getInstance().schedule(
    new WaitCommand(1).andThen(
        new InstantCommand(() ->
            m_robotContainer.getVision().setVisionEnabled(true) 
        )
    )
    );

    m_robotContainer.getVision().getEstimatedPose().ifPresent(pose -> {
    m_robotContainer.getDrivetrain().resetPose(pose); 
    });
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
