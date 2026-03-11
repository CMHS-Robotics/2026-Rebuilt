package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase; // Use this instead of "Robot"
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.tools.FieldUtil;

import org.littletonrobotics.junction.Logger;


public class Vision extends SubsystemBase {

    private final Translation2d hubOffset = new Translation2d(.6,0);

    private boolean allowVisionFusion = false;
    private final CommandSwerveDrivetrain swerve;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;

    private VisionSystemSim visionSim;
    private PhotonCameraSim[] cameraSims;
    
    private Matrix<N3, N1> curStdDevs;

    private final Translation2d shooterOffset = new Translation2d(0.2921, -0.1);

    public Vision(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;


        // Initialize arrays for 2 cameras
        cameras = new PhotonCamera[] {
            new PhotonCamera(frontLeftCam),
            new PhotonCamera(frontRightCam)
        };

        estimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(kTagLayout, kRobotToFrontLeftCam),
            new PhotonPoseEstimator(kTagLayout, kRobotToFrontRightCam)
        };


       // SmartDashboard.putData("Vision Field", fieldVisualizer);

         if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(kTagLayout);

            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setFPS(45);
            cameraProp.setAvgLatencyMs(50);

            // Create a sim for EACH camera
            cameraSims = new PhotonCameraSim[cameras.length];
            
            // Link Sim 0 to Camera 0 (Front Left)
            cameraSims[0] = new PhotonCameraSim(cameras[0], cameraProp);
            visionSim.addCamera(cameraSims[0], kRobotToFrontLeftCam);

            // Link Sim 1 to Camera 1 (Front Right)
            cameraSims[1] = new PhotonCameraSim(cameras[1], cameraProp);
            visionSim.addCamera(cameraSims[1], kRobotToFrontRightCam);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the vision simulation with the ACTUAL robot pose from the drivetrain
        visionSim.update(swerve.getState().Pose);
    }

    @Override
public void periodic() {
    EstimatedRobotPose bestEstimate = null;
    Matrix<N3, N1> bestStdDevs = null;
    double bestScore = Double.MAX_VALUE; // Lower score is better (less uncertainty)

    boolean sawTagThisFrame = false;

    // 1. Iterate through all cameras and unread results
    for (int i = 0; i < cameras.length; i++) {
        PhotonCamera cam = cameras[i];
        PhotonPoseEstimator estimator = estimators[i];

        for (var result : cam.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            // Try Multi-tag first, fall back to lowest ambiguity single tag
            Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEst.isPresent()) {
                EstimatedRobotPose est = visionEst.get();
                
                // Calculate uncertainty for THIS specific result
                updateEstimationStdDevs(visionEst, result.getTargets(), estimator);
                
                // Scoring: We use the sum of X and Y standard deviations
                // A smaller value means the pose is more trustworthy
                double currentScore = curStdDevs.get(0, 0) + curStdDevs.get(1, 0);

                if (currentScore < bestScore) {
                    bestScore = currentScore;
                    bestEstimate = est;
                    bestStdDevs = curStdDevs;
                }
                sawTagThisFrame = true;
            }
        }
    }

    
    // 2. Fusion: Only add the absolute best estimate found across all cameras
    if (allowVisionFusion && bestEstimate != null) {
        swerve.addVisionMeasurement(
            bestEstimate.estimatedPose.toPose2d(),
            bestEstimate.timestampSeconds,
            bestStdDevs
        );
        
        Logger.recordOutput("Pose/VisionEstimate", bestEstimate.estimatedPose.toPose2d());
    }

    // 3. Logging & Debugging
    if (Robot.isSimulation() && bestEstimate != null) {
        getSimDebugField().getObject("VisionEstimation").setPose(bestEstimate.estimatedPose.toPose2d());
    } else if (Robot.isSimulation()) {
        getSimDebugField().getObject("VisionEstimation").setPoses();
    }

    Logger.recordOutput("Pose/Robot", swerve.getState().Pose);
    Logger.recordOutput("Vision/FusionEnabled", allowVisionFusion);
    Logger.recordOutput("Vision/SawTag", sawTagThisFrame);
}

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
        
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = kSingleTagStdDevs;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                
                // If only one tag and it's too far, don't trust it at all (Double.MAX_VALUE)
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else 
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                
                curStdDevs = estStdDevs;
            }
        }
    }

    // --- Pose Analysis Methods ---

   public Optional<Rotation2d> getRotationErrorRobotToTagFromPose(int tagId) {
    Pose2d robotPose = swerve.getState().Pose;
    
    return kTagLayout.getTagPose(tagId).map(tagPose3d -> {
        // 1. Calculate the Target Point (0.6m back from the tag face)
        Transform3d tagPullback = new Transform3d(new Translation3d(-0.6, 0, 0), new Rotation3d());
        Pose2d targetPose = tagPose3d.plus(tagPullback).toPose2d();

        // 2. Calculate the Shooter's position in the Field Frame
        // Convert shooterOffset (robot-relative) to field-relative using robot rotation
        Translation2d shooterFieldOffset = shooterOffset.rotateBy(robotPose.getRotation());
        Translation2d shooterWorldPos = robotPose.getTranslation().plus(shooterFieldOffset);

        // 3. Calculate the vector from the SHOOTER to the TARGET
        Translation2d vectorShooterToTarget = targetPose.getTranslation().minus(shooterWorldPos);
        
        // 4. Return the rotation error
        // The angle the robot needs to point so the shooter (at its offset) faces the target
        return vectorShooterToTarget.getAngle().minus(robotPose.getRotation());
    });
}

    

    public Optional<Double> distanceToTagFromPose(int tagId) {

        Optional<Pose3d> tagPoseOpt = kTagLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();
    
        Pose2d robotPose = swerve.getState().Pose;
    
        Translation2d tagTranslation =
            tagPoseOpt.get().toPose2d().getTranslation();
    
        // Rotate shooter offset into field frame
        Translation2d shooterFieldOffset = shooterOffset.rotateBy(robotPose.getRotation());
    
        // Shooter world position
         Translation2d shooterWorldPos = robotPose.getTranslation().plus(shooterFieldOffset);
    
        return Optional.of(
            shooterWorldPos.getDistance(tagTranslation)
        );
    }

    public Optional<Double> distanceToHubFromPose() {

    int tagId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26; 
    
    Pose2d robotPose = swerve.getState().Pose;

    Optional<Translation2d> hubOpt =
        FieldUtil.getHubTranslation(kTagLayout, tagId);

    if (hubOpt.isEmpty()) return Optional.empty();

    Translation2d hubTranslation = hubOpt.get();

    Translation2d shooterFieldOffset =
        shooterOffset.rotateBy(robotPose.getRotation());

    Translation2d shooterWorldPos =
        robotPose.getTranslation().plus(shooterFieldOffset);

    return Optional.of(
        shooterWorldPos.getDistance(hubTranslation)
    );
}

    public Optional<Translation2d> translationShooterToTagFromPose(int tagId) {
        return kTagLayout.getTagPose(tagId).map(tagPose -> {
            Pose2d robotPose = swerve.getState().Pose;
            
            // Convert shooter offset from robot-relative to field-relative
            Translation2d shooterFieldOffset = shooterOffset.rotateBy(robotPose.getRotation());
            
            // Calculate shooter's current position on the field
            Translation2d shooterWorldPos = robotPose.getTranslation().plus(shooterFieldOffset);
            
            // Return vector from shooter to tag
            return tagPose.toPose2d().getTranslation().minus(shooterWorldPos);
        });
    }



    public Optional<Rotation2d> getRawRotationErrorToHubAnyTag() {
    // 1. Determine target Hub based on Alliance
    Transform3d[] robotToCamTransforms = {kRobotToFrontRightCam, kRobotToFrontLeftCam};

    int targetHubId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;
    Optional<Pose3d> hubPoseOpt = kTagLayout.getTagPose(targetHubId);
    
    if (hubPoseOpt.isEmpty()) return Optional.empty();
    Pose3d hubFieldPose = hubPoseOpt.get();

    for (int i = 0; i < cameras.length; i++) {
        var result = cameras[i].getLatestResult();
        if (!result.hasTargets()) continue;

        // 2. Pick the best target currently seen by this camera
        PhotonTrackedTarget seenTarget = result.getBestTarget();
        int seenTagId = seenTarget.getFiducialId();
        
        Optional<Pose3d> seenTagFieldPoseOpt = kTagLayout.getTagPose(seenTagId);
        if (seenTagFieldPoseOpt.isEmpty()) continue;
        
        // 3. Math: Find where the Hub is relative to the Tag we see
        // Transformation: HubRelative = Inverse(SeenTagFieldPose) * HubFieldPose
        Pose3d seenTagFieldPose = seenTagFieldPoseOpt.get();
        Transform3d tagToHub = new Transform3d(seenTagFieldPose, hubFieldPose);

        // 4. Math: Find where the Hub is relative to the Camera
        // CamToHub = CamToSeenTag * TagToHub
        Transform3d camToSeenTag = seenTarget.getBestCameraToTarget();
        Transform3d camToHub = camToSeenTag.plus(tagToHub);

        // 5. Math: Find where the Hub is relative to the Robot Center
        Transform3d robotToHub = robotToCamTransforms[i].plus(camToHub);

        // 6. Math: Adjust for Shooter Offset
        Translation2d shooterToHub = robotToHub.getTranslation().toTranslation2d().minus(shooterOffset);

        // 7. Return the raw angle to the Hub
        return Optional.of(new Rotation2d(shooterToHub.getX(), shooterToHub.getY()));
    }
    return Optional.empty();
    }

    public Optional<Translation2d> getRawTranslationShooterToHubAnyTag() {
    Transform3d[] robotToCamTransforms = {kRobotToFrontRightCam, kRobotToFrontLeftCam};
    // 1. Identify the target Hub ID based on Alliance
    int targetHubId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;
    Optional<Pose3d> hubPoseOpt = kTagLayout.getTagPose(targetHubId);
    if (hubPoseOpt.isEmpty()) return Optional.empty();
    Pose3d hubFieldPose = hubPoseOpt.get();

    for (int i = 0; i < 1; i++) {
        var result = cameras[i].getLatestResult();
        if (!result.hasTargets()) continue;

        // 2. Use the best visible tag as a reference point
        PhotonTrackedTarget seenTarget = result.getBestTarget();
        Optional<Pose3d> seenTagFieldPoseOpt = kTagLayout.getTagPose(seenTarget.getFiducialId());
        if (seenTagFieldPoseOpt.isEmpty()) continue;

        // 3. Geometric Chain:
        // TagToHub = (Field->Tag)^-1 * (Field->Hub)
        Transform3d tagToHub = new Transform3d(seenTagFieldPoseOpt.get(), hubFieldPose);
        
        // CamToHub = CamToSeenTag + TagToHub
        Transform3d camToHub = seenTarget.getBestCameraToTarget().plus(tagToHub);
        
        // RobotToHub = RobotToCam + CamToHub
        Transform3d robotToHub = robotToCamTransforms[i].plus(camToHub);

        // 4. Final step: Subtract shooter offset (Robot Center -> Shooter)
        // Result is the vector from the Shooter to the Hub in Robot-Relative Space
        return Optional.of(robotToHub.getTranslation().toTranslation2d().minus(shooterOffset));
    }
    return Optional.empty();
    }

    public Optional<Double> getRawDistanceShooterToHubAnyTag() {
    return getRawTranslationShooterToHubAnyTag().map(Translation2d::getNorm);
}

/**
 * Calculates the rotation the ROBOT must achieve for the SHOOTER to face the Hub
 * using any visible tag as a reference.
 */
public Optional<Rotation2d> getRawRotationShooterToHubAnyTag() {
    return getRawTranslationShooterToHubAnyTag().map(translation -> 
        new Rotation2d(translation.getX(), translation.getY())
    );
}






    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }


    public void setVisionEnabled(boolean enabled) {
             allowVisionFusion = enabled;
         }
}