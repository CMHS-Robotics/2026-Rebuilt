package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {

    // Four cameras
    private final PhotonCamera camFL = new PhotonCamera("FrontLeftCamera");
    private final PhotonCamera camFR = new PhotonCamera("FrontRightCamera");
    private final PhotonCamera camBL = new PhotonCamera("BackLeftCamera");
    private final PhotonCamera camBR = new PhotonCamera("BackRightCamera");

    // Camera -> Robot transforms
    private static final Transform3d kRobotToFL = new Transform3d(0.3625, 0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(15)));
    private static final Transform3d kRobotToFR = new Transform3d(0.3625, -0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(-15)));
    private static final Transform3d kRobotToBL = new Transform3d(-0.3625, 0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(165)));
    private static final Transform3d kRobotToBR = new Transform3d(-0.3625, -0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(-165)));

    private final CommandSwerveDrivetrain swerve;
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estFL;
    private final PhotonPoseEstimator estFR;
    private final PhotonPoseEstimator estBL;
    private final PhotonPoseEstimator estBR;

    private final Field2d fieldVisualizer = new Field2d();

    private Pose2d latestFieldPose = new Pose2d();

    public Vision(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout layout) {
        this.swerve = drivetrain;
        this.fieldLayout = layout;

        estFL = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToFL.inverse());
        estFR = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToFR.inverse());
        estBL = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToBL.inverse());
        estBR = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToBR.inverse());

        fieldVisualizer.setRobotPose(latestFieldPose);
    }

    @Override
    public void periodic() {
        Pose2d odomPose = swerve.getState().Pose;

        // Set reference pose for all cameras
        estFL.setReferencePose(odomPose);
        estFR.setReferencePose(odomPose);
        estBL.setReferencePose(odomPose);
        estBR.setReferencePose(odomPose);

        // Get estimated poses
        Optional<EstimatedRobotPose> poseFL = estFL.update(camFL.getLatestResult());
        Optional<EstimatedRobotPose> poseFR = estFR.update(camFR.getLatestResult());
        Optional<EstimatedRobotPose> poseBL = estBL.update(camBL.getLatestResult());
        Optional<EstimatedRobotPose> poseBR = estBR.update(camBR.getLatestResult());

        // Collect valid poses and weights (weight = # of visible tags)
        List<EstimatedRobotPose> poses = new ArrayList<>();
        List<Integer> weights = new ArrayList<>();

        addPoseIfValid(poseFL, camFL, poses, weights);
        addPoseIfValid(poseFR, camFR, poses, weights);
        addPoseIfValid(poseBL, camBL, poses, weights);
        addPoseIfValid(poseBR, camBR, poses, weights);

        if (!poses.isEmpty()) {
            double totalWeight = 0;
            double x = 0, y = 0;
            double cosSum = 0, sinSum = 0;
            double timestamp = 0;

            for (int i = 0; i < poses.size(); i++) {
                EstimatedRobotPose p = poses.get(i);
                int w = weights.get(i);

                Pose2d pose2d = p.estimatedPose.toPose2d();

                // Outlier rejection: ignore poses > 1.0m from odometry
                if (pose2d.getTranslation().getDistance(odomPose.getTranslation()) > 1.0) continue;

                x += pose2d.getX() * w;
                y += pose2d.getY() * w;
                cosSum += Math.cos(pose2d.getRotation().getRadians()) * w;
                sinSum += Math.sin(pose2d.getRotation().getRadians()) * w;
                timestamp += p.timestampSeconds * w;

                totalWeight += w;
            }

            if (totalWeight > 0) {
                Pose2d fused = new Pose2d(
                        x / totalWeight,
                        y / totalWeight,
                        new Rotation2d(cosSum / totalWeight, sinSum / totalWeight)
                );

                // Feed into drivetrain for fusion
                swerve.addVisionMeasurement(fused, timestamp / totalWeight);
                latestFieldPose = fused;
            }
        }

        fieldVisualizer.setRobotPose(latestFieldPose);
    }

    private void addPoseIfValid(Optional<EstimatedRobotPose> poseOpt, PhotonCamera cam,
                                List<EstimatedRobotPose> poses, List<Integer> weights) {
        if (poseOpt.isPresent()) {
            poses.add(poseOpt.get());
            // Weight = number of visible targets for this camera
            weights.add(cam.getLatestResult().getTargets().size());
        }
    }

    // --- Helper function: distance to a specific tag ID ---
    public Optional<Double> distanceToTag(int tagId) {
        // Check all cameras for the tag
        List<PhotonCamera> cameras = List.of(camFL, camFR, camBL, camBR);
    
        for (PhotonCamera cam : cameras) {
            var result = cam.getLatestResult();
            if (!result.hasTargets()) continue;
    
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.getFiducialId() == tagId) {
                    Transform3d camToTag = t.getBestCameraToTarget();
                    // Planar distance (ignore Z)
                    return Optional.of(Math.hypot(camToTag.getX(), camToTag.getY()));
                }
            }
        }
    
        return Optional.empty();
    }
}