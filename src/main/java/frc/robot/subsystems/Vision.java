package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase; // Use this instead of "Robot"
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {

    private boolean allowVisionFusion = false;
    private final CommandSwerveDrivetrain swerve;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;
    private Matrix<N3, N1> curStdDevs;
    private final Field2d fieldVisualizer = new Field2d();

    private final Translation2d shooterOffset = new Translation2d(0.2921, -0.2286);

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
        SmartDashboard.putData("Vision Field", fieldVisualizer);
    }

    public void periodic() {
        Pose2d odomPose = swerve.getState().Pose;
        SmartDashboard.putNumber("Robot Heading", odomPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("front right has targets?", cameras[1].getLatestResult().hasTargets());
        SmartDashboard.putBoolean("front left has targets?", cameras[0].getLatestResult().hasTargets());
        getRawRotationErrorToHubAnyTag().ifPresent(rot -> SmartDashboard.putNumber("Angle To Hub from shooter from direct", rot.getDegrees()));
        distanceToTagFromPose(10).ifPresent(distance -> SmartDashboard.putNumber("distance direct to hub", distance));
        distanceToTagFromPose(10).ifPresent(distance -> SmartDashboard.putNumber("distance fromPose to hub", distance));
        getRotationErrorRobotToTagFromPose(10).ifPresent(rot -> SmartDashboard.putNumber("Angle To Hub from shooter from pose", rot.getDegrees()));



        boolean sawTagThisFrame = false;

        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera cam = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];

            for (var result : cam.getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }

                updateEstimationStdDevs(visionEst, result.getTargets(), estimator);


                if(allowVisionFusion){
                     visionEst.ifPresent(est -> {
                    swerve.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), 
                        est.timestampSeconds, 
                        curStdDevs
                    );
                });
                sawTagThisFrame = true;
                }
            }
        }

        // --- Move these outside the loop so they update every 20ms regardless of vision ---
        fieldVisualizer.setRobotPose(swerve.getState().Pose);
        SmartDashboard.putBoolean("V fused", sawTagThisFrame);
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
        return kTagLayout.getTagPose(tagId).map(tagPose -> {
            Translation2d vectorToHub = tagPose.toPose2d().getTranslation().minus(robotPose.getTranslation());
            return vectorToHub.getAngle().minus(robotPose.getRotation());
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

    for (int i = 0; i < cameras.length; i++) {
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


    public void setVisionEnabled(boolean enabled) {
             allowVisionFusion = enabled;
         }
}