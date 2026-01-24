// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  // Shooter motor (Kraken / TalonFX)
  private final TalonFX shooterMotor = new TalonFX(1);

  // Velocity control request
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Ramp: 3000 RPM in 3 seconds → 1000 RPM/sec
  private final SlewRateLimiter rpmRamp = new SlewRateLimiter(1000.0);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- PID + Feedforward (STARTING VALUES, MUST TUNE) ---
    config.Slot0.kP = 0.12;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;

    // Optional: ensure correct direction
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterMotor.getConfigurator().apply(config);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Reset ramp so it always starts from 0 RPM
    rpmRamp.reset(0.0);
  }

  @Override
  public void teleopPeriodic() {

    // === SET YOUR TARGET RPM HERE ===
    double targetRPM = 6000.0;

    // Smooth ramp
    double rampedRPM = rpmRamp.calculate(targetRPM);

    // Phoenix expects rotations per second
    double targetRPS = rampedRPM / 60.0;

    shooterMotor.setControl(
        velocityRequest.withVelocity(targetRPS)
    );
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
