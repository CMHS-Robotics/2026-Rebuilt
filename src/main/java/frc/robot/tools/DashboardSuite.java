package frc.robot.tools;

//import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DashboardSuite extends SubsystemBase{
    
    //subsystems
  
    //subscribers
        //subscribers will monitor data in a topic and can be used to update code based on inputs in elastic

        //elevator
    DoubleSubscriber sElevatorPIDBar;
    BooleanSubscriber sElevatorPIDManual;
    DoubleSubscriber sElevatorPIDP;
    DoubleSubscriber sElevatorPIDD;
    DoubleSubscriber sElevatorPIDClampUpper;
    DoubleSubscriber sElevatorPIDClampLower;
        //vision
    IntegerSubscriber sVisionTargetId;
    IntegerSubscriber sVisionTargetMode;


    //publishers
        //publishers will publish data to a topic and update the values in elastic

        //elevator
    DoublePublisher pElevatorLeftMotor;
    DoublePublisher pElevatorRightMotor;
    StringPublisher pElevatorPID;
    StringPublisher pElevatorPIDSettings;
    BooleanPublisher pElevatorHasReached;
    DoublePublisher pElevatorPIDResult;
    DoublePublisher pElevatorPIDPreClampResult;
        //coral
    DoublePublisher pCoralWristMotor;
    DoublePublisher pCoralWristOutput;
    StringPublisher pCoralWristPID;
    BooleanPublisher pCoralWristHasReached;
        //vision
    BooleanPublisher pVisionHasDetected;
    IntegerPublisher pAprilTagDetected;
    BooleanPublisher pVisionHasTarget;
    StringPublisher pVisionTargetMode;
        //visionv2
    IntegerPublisher pSameTargetID;
    BooleanPublisher pHasSameTarget;
    
   // public DashboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w,Vision v,VisionV2 v2){
   //     Elevator = e;
   //     CoralSpin = s;
   //     CoralWrist = w;
   //     Vision = v;
   //     VisionV2 = v2;
   //     initialize();
   // }

    public final void initialize(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        //elevator data
        NetworkTable ElevatorData = inst.getTable("Elevator");


        pElevatorLeftMotor = ElevatorData.getDoubleTopic("Elevator Left Motor").publish();
        pElevatorRightMotor = ElevatorData.getDoubleTopic("Elevator Right Motor").publish();
        pElevatorPID = ElevatorData.getStringTopic("Elevator PID").publish();
        pElevatorPIDSettings = ElevatorData.getStringTopic("Elevator PID Settings").publish();
        pElevatorPIDResult = ElevatorData.getDoubleTopic("Elevator PID Result").publish();
        pElevatorPIDPreClampResult = ElevatorData.getDoubleTopic("Elevator PID Pre Clamp Result").publish();
        pElevatorHasReached = ElevatorData.getBooleanTopic("Elevator Has Reached Target").publish();

        NetworkTable ElevatorManualPIDTargetting = inst.getTable("Elevator PID Manual");

        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Bar Manual Target").publish().set(0.0);
        ElevatorManualPIDTargetting.getBooleanTopic("Elevator Manual Control Active").publish().set(false);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID P Value").publish().set(0.2);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID D Value").publish().set(0.4);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Lower Limit").publish().set(-0.15);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Upper Limit").publish().set(0.4);


        sElevatorPIDBar = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Bar Manual Target").subscribe(0.0);
        sElevatorPIDManual = ElevatorManualPIDTargetting.getBooleanTopic("Elevator Manual Control Active").subscribe(false);
        sElevatorPIDP = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID P Value").subscribe(0);
        sElevatorPIDD = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID D Value").subscribe(0);
        sElevatorPIDClampLower = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Lower Limit").subscribe(0);
        sElevatorPIDClampUpper = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Upper Limit").subscribe(0);

        //coral data
        NetworkTable CoralData = inst.getTable("Coral");

        pCoralWristPID = CoralData.getStringTopic("Elevator PID").publish();
        pCoralWristMotor = CoralData.getDoubleTopic("Coral Wrist Position").publish();
        pCoralWristOutput = CoralData.getDoubleTopic("Coral Wrist Output").publish();
        pCoralWristHasReached = CoralData.getBooleanTopic("Wrist Has Reached Target").publish();

        //vision data
        NetworkTable VisionData = inst.getTable("Vision");

        VisionData.getIntegerTopic("Vision April Tag ID Target").publish().set(0);
        VisionData.getIntegerTopic("Vision Target Mode Set").publish().set(0);
        pAprilTagDetected = VisionData.getIntegerTopic("ID of April Tag Detected").publish();
        pVisionHasDetected = VisionData.getBooleanTopic("Vision Has Detected Target").publish();
        pVisionHasTarget = VisionData.getBooleanTopic("Vision Has Target").publish();
        pVisionTargetMode = VisionData.getStringTopic("Vision Target Mode").publish();

        sVisionTargetId = VisionData.getIntegerTopic("Vision April Tag ID Target").subscribe(0);
        sVisionTargetMode = VisionData.getIntegerTopic("Vision Target Mode Set").subscribe(0);
        //vision v2

        NetworkTable VisionV2Data = inst.getTable("VisionV2"); 
        VisionV2Data.getIntegerTopic("SameTargetId").publish().set(-1);
        VisionV2Data.getBooleanTopic("HasSameTargetId").publish().set(false);
        pSameTargetID = VisionV2Data.getIntegerTopic("SameTargetId").publish();
        pHasSameTarget = VisionV2Data.getBooleanTopic("HasSameTargetId").publish();

        //other
        SmartDashboard.putNumber("Match Time",DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());

        //commands
            //these commands can be activated from elastic
        
    }


    @Override
    public void periodic(){

        //elevator

    
    
    }
}
