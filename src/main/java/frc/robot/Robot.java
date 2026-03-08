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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;



  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

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
     CommandScheduler.getInstance().schedule(m_autonomousCommand);

    m_robotContainer.getVision().setVisionEnabled(false);

    CommandScheduler.getInstance().schedule(
    new WaitCommand(1).andThen(
        new InstantCommand(() ->
            m_robotContainer.getVision().setVisionEnabled(true) 
        )
    )
    );
    }

   // Pose2d startingPose = m_autonomousCommand.getStartingPose();
   //swerve.resetPose(startingPose);
  }
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
  public void simulationPeriodic() {}
}
