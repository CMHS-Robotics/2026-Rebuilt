package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import  edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor = new TalonFX(14); //reset this to wtv it is gonna be

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final SlewRateLimiter rpmRamp = new SlewRateLimiter(2000.0);
    private final double RPM_TOLLERANCE = 300;
    private double targetRPM = 0;
    private final Vision vision;

    public Shooter(Vision vision) {
        this.vision = vision;
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Increase P to fight friction at low speeds and pull back overshoots at high speeds
        config.Slot0.kP = 0.35; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.01; // A tiny bit of D can help prevent jitter

        // Lower V slightly if you are consistently overshooting your targets
        config.Slot0.kV = 0.125;    

        // OPTIONAL: Add kS (Static Feedforward) 
        // kS overcomes friction. If it takes 0.2V just to get the motor moving, set kS to 0.2.
        config.Slot0.kS = 0.1;
        //config.CurrentLimits.StatorCurrentLimit = 80;
        //config.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
         double rampedRPM = rpmRamp.calculate(rpm);
         double targetRPS = rampedRPM / 60.0;
         shooterMotor.setControl(velocityRequest.withVelocity(targetRPS));
         this.targetRPM = rpm;

        
    }

    public void stop() {
        shooterMotor.set(0);
    }


    public void resetRamp() {
        rpmRamp.reset(0.0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
      }

    public double getHoodAngleDegrees() {
            //return hoodMotor.getPosition().getValueAsDouble() * DEGREES_PER_MOTOR_ROTATION;
            return 0.0;
}

    @Override
    public void periodic() {
    // Correct way to get velocity in Phoenix 6
    // .getValueAsDouble() returns Rotations per Second (RPS)
    double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
    double currentRPM = currentRPS * 60.0;
    double target = SmartDashboard.getNumber("SetRPM", 0);

    double shooter1Error = target - currentRPM;

    SmartDashboard.putNumber("ShooterMotor Error", shooter1Error);

    SmartDashboard.putNumber("ShooterMotor RPM", currentRPM);
}


    public boolean atSetPoint(){
        
        double currentRPM = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
        
        // Use Math.abs to check if the difference is within the tolerance range
        if ((Math.abs(currentRPM - targetRPM) <= RPM_TOLLERANCE)) {//&& (vision.getRotationErrorShooterToTagFromPose().get().getDegrees()  < ShooterMath.getShotTolerance(vision.getBestHubDistance())) ){
            return true;
        }
        else return false;
    }
}