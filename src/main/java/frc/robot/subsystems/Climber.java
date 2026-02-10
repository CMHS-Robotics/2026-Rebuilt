package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.tools.PID;


public class Climber extends SubsystemBase {
    TalonFX climberMotor = new TalonFX(15);
    TalonFXConfiguration config = new TalonFXConfiguration();
    public double []stages = new double[3];
    public double []speeds = new double[2];
    PID ElevatorPID;
    double position;
    double positionTolerence;
    public FreeMoveStates currentFreeMoveState;

    public Climber() {
        //Config
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;
        //Config Software Limits
        // var softwarelimit = config.SoftwareLimitSwitch;
        // softwarelimit.ForwardSoftLimitEnable = true;
        // softwarelimit.ReverseSoftLimitEnable = true;
        // softwarelimit.ReverseSoftLimitThreshold = 1;
        // softwarelimit.ForwardSoftLimitThreshold = 24.2;

        climberMotor.getConfigurator().apply(config);

        //PID
        ElevatorPID = new PID(0.1,0.05,0.01);
        ElevatorPID.setGravity(1.2);//Counteracts Weight Of Robot Against Gravity
        ElevatorPID.setMaxInput(0.4);

        //Climber Stages
        stages[0] = 0; //Base Level
        stages[1] = 0.80*16;//Ground To First Bar 
        stages[2] = 0.30*16;//Bar To Bar

        //Climber Speeds
        speeds[0] = 0.1;//Up Speed
        speeds[1] = -0.1;//Down Speed

        //Climb Position Tolerence
        positionTolerence = 0.1;

        currentFreeMoveState = FreeMoveStates.Disabled;

        ZeroClimber();
    }
    //States For FreeMove
    public enum FreeMoveStates {
        Enabled,//Specifies That We Have Control, Can Only Move From Enabled To Positive/Disabled
        Disabled,
        PositiveDirection,
        NegativeDirection
    }

    public String FreeMoveStateInterpreter(FreeMoveStates s) {
        return s.name();
    }
    public FreeMoveStates FreeMoveStateInterpreter(String s){
        return FreeMoveStates.valueOf(s);
    }

    public void moveUp(){
        climberMotor.set(speeds[0]);
    }

    public void moveDown(){
        climberMotor.set(speeds[1]);
    }

    public void stopClimber(){
        climberMotor.set(0);
    }

    public double getPosition(){
        return climberMotor.getPosition().getValueAsDouble();
    }
    public boolean atPosition(double target){
        return Math.abs(getPosition() - target) <= positionTolerence;
    }

    public void moveToward(double target){
        double error = target - getPosition();

        if(Math.abs(error) <= positionTolerence){
            stopClimber();
        }else if(error > 0 ){
            moveUp();
        }else{
            moveDown();
        }
    }
    public void SetStage(double s) {
        moveToward(s);
    }
    public void ZeroClimber() {
        climberMotor.setPosition(0);
    }
@Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());
        SmartDashboard.setDefaultString("Free Move Climber State", FreeMoveStateInterpreter(currentFreeMoveState));
    }
}

