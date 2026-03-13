package frc.robot.commands;
import frc.robot.subsystems.*;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DriveToClimbPose extends Command {

  private final Pose2d targetPose;
  private final CommandSwerveDrivetrain drivetrain;

  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  private final SwerveRequest.FieldCentric zero =
      new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0);

  private final PIDController xController = new PIDController(1.5, 0, 0);
  private final PIDController yController = new PIDController(1.5, 0, 0);
  private final PIDController thetaController = new PIDController(2.5, 0, 0);

  public DriveToClimbPose(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
      this.drivetrain = drivetrain;
      this.targetPose = targetPose;

      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      xController.setTolerance(0.025);      // ~1 inch
      yController.setTolerance(0.025);
      thetaController.setTolerance(Math.toRadians(2));
  }

  @Override
  public void execute() {
      Pose2d current = drivetrain.getPose();

      double xSpeed = xController.calculate(current.getX(), targetPose.getX());
      double ySpeed = yController.calculate(current.getY(), targetPose.getY());
      double thetaSpeed = thetaController.calculate(current.getRotation().getRadians(), targetPose.getRotation().getRadians());
      xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
      ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
      thetaSpeed = MathUtil.clamp(thetaSpeed, -2.0, 2.0);

      drivetrain.setControl(request.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(thetaSpeed));
  }

  @Override
  public void end(boolean interrupted) {
      drivetrain.setControl(zero);
  }

  @Override
  public boolean isFinished() {
      return xController.atSetpoint()&& yController.atSetpoint() && thetaController.atSetpoint();
  }
}