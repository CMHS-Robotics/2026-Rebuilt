package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;

public class Vision extends SubsystemBase {

    // Four cameras
    private final PhotonCamera frontCam     = new PhotonCamera("FrontLeftCamera");
    private final PhotonCamera leftBackCam  = new PhotonCamera("FrontRightCamera");
    private final PhotonCamera leftFrontCam = new PhotonCamera("BackLeftCamera");
    private final PhotonCamera rightCam     = new PhotonCamera("BackRightCamera");

    // Camera → Robot transforms
    private static final Transform3d kRobotToFrontCam = new Transform3d(
        0.3625,  0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(15))
    );
    private static final Transform3d kRobotToLeftBackCam = new Transform3d(
        0.3625, -0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(-15))
    );
    private static final Transform3d kRobotToLeftFrontCam = new Transform3d(
       -0.3625,  0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(165))
    );
    private static final Transform3d kRobotToRightCam = new Transform3d(
       -0.3625, -0.22, 0.12, new Rotation3d(0, 0, Math.toRadians(-165))
    );

    private final CommandSwerveDrivetrain swerve;
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estFront;
    private final PhotonPoseEstimator estLeftBack;
    private final PhotonPoseEstimator estLeftFront;
    private final PhotonPoseEstimator estRight;

    private final Field2d fieldVisualizer = new Field2d();
    private Pose2d latestFieldPose = new Pose2d();

    public Vision(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout layout) {
        this.swerve = drivetrain;
        this.fieldLayout = layout;

        // IMPORTANT: Pass the PhotonCamera instance into the PhotonPoseEstimator
        estFront     = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,     kRobotToFrontCam);
        estLeftBack  = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftBackCam,  kRobotToLeftBackCam);
        estLeftFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftFrontCam, kRobotToLeftFrontCam);
        estRight     = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,     kRobotToRightCam);

        fieldVisualizer.setRobotPose(latestFieldPose);
    }

    @Override
    public void periodic() {
        Pose2d odomPose = swerve.getState().Pose;

        estFront.setReferencePose(odomPose);
        estLeftBack.setReferencePose(odomPose);
        estLeftFront.setReferencePose(odomPose);
        estRight.setReferencePose(odomPose);

        Optional<EstimatedRobotPose> poseFront     = estFront.update(frontCam.getLatestResult());
        Optional<EstimatedRobotPose> poseLeftBack  = estLeftBack.update(leftBackCam.getLatestResult());
        Optional<EstimatedRobotPose> poseLeftFront = estLeftFront.update(leftFrontCam.getLatestResult());
        Optional<EstimatedRobotPose> poseRight     = estRight.update(rightCam.getLatestResult());

        List<EstimatedRobotPose> poses = new ArrayList<>();
        List<Integer> weights = new ArrayList<>();
        List<PhotonCamera> cams = new ArrayList<>();

        addPoseIfValid(poseFront,     frontCam,     poses, weights, cams);
        addPoseIfValid(poseLeftBack,  leftBackCam,  poses, weights, cams);
        addPoseIfValid(poseLeftFront, leftFrontCam, poses, weights, cams);
        addPoseIfValid(poseRight,     rightCam,     poses, weights, cams);

        if (poses.isEmpty()) return;

        double totalWeight = 0;
        double x = 0, y = 0;
        double cosSum = 0, sinSum = 0;
        double timestamp = 0;

        double totalTags = 0;
        double totalDist = 0;

        for (int i = 0; i < poses.size(); i++) {
            EstimatedRobotPose p = poses.get(i);
            int w = weights.get(i);
            PhotonCamera cam = cams.get(i);

            Pose2d pose2d = p.estimatedPose.toPose2d();

            if (pose2d.getTranslation().getDistance(odomPose.getTranslation()) > 1.0)
                continue;

            x += pose2d.getX() * w;
            y += pose2d.getY() * w;
            cosSum += Math.cos(pose2d.getRotation().getRadians()) * w;
            sinSum += Math.sin(pose2d.getRotation().getRadians()) * w;
            timestamp += p.timestampSeconds * w;

            totalWeight += w;

            var targets = cam.getLatestResult().getTargets();
            totalTags += targets.size();

            for (var t : targets) {
                totalDist += t.getBestCameraToTarget().getTranslation().getNorm();
            }
        }

        if (totalWeight <= 0 || totalTags <= 0) return;

        Pose2d fused = new Pose2d(
            x / totalWeight,
            y / totalWeight,
            new Rotation2d(cosSum / totalWeight, sinSum / totalWeight)
        );

        double avgDist = totalDist / totalTags;

        // ---- STD DEV CALC (simple version) ----
        double xyStd = 0.1 + 0.05 * avgDist;
        double thetaStd = 0.2 + 0.1 * avgDist;

        if (totalTags <= 1) {
            xyStd *= 2.0;
            thetaStd *= 2.5;
        }

        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, thetaStd);

        swerve.addVisionMeasurement(
            fused,
            timestamp / totalWeight,
            stdDevs
        );

        latestFieldPose = fused;
        fieldVisualizer.setRobotPose(latestFieldPose);
    }

    private void addPoseIfValid(
        Optional<EstimatedRobotPose> poseOpt,
        PhotonCamera cam,
        List<EstimatedRobotPose> poses,
        List<Integer> weights,
        List<PhotonCamera> cams
    ) {
        if (poseOpt.isPresent() && cam.getLatestResult().hasTargets()) {
            poses.add(poseOpt.get());
            weights.add(cam.getLatestResult().getTargets().size());
            cams.add(cam);
        }
    }

    // --- Helper function: distance to a specific tag ID ---
    public Optional<Double> distanceToTag(int tagId) {
        List<PhotonCamera> cameras = List.of(frontCam, leftBackCam, leftFrontCam, rightCam);

        for (PhotonCamera cam : cameras) {
            var result = cam.getLatestResult();
            if (!result.hasTargets()) continue;

            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.getFiducialId() == tagId) {
                    var camToTag = t.getBestCameraToTarget();
                    return Optional.of(Math.hypot(camToTag.getX(), camToTag.getY()));
                }
            }
        }

        return Optional.empty();
    }

    public Optional<Double> distanceToTagFromPose(int tagId) {
        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();

        Pose2d robotPose = latestFieldPose;
        Translation2d tagTranslation = tagPoseOpt.get().toPose2d().getTranslation();

        return Optional.of(robotPose.getTranslation().getDistance(tagTranslation));
    }

    public Optional<Translation2d> translationToTagFromPose(int tagId) {
        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();

        Pose2d robotPose = latestFieldPose;
        Translation2d tagTranslation = tagPoseOpt.get().toPose2d().getTranslation();

        return Optional.of(tagTranslation.minus(robotPose.getTranslation()));
    }

    public Optional<Rotation2d> getRotationErrorToTag(int tagId) {
        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();

        Pose2d robotPose = swerve.getState().Pose;
        Translation2d tagPos = tagPoseOpt.get().toPose2d().getTranslation();
        Rotation2d angleToTag = tagPos.minus(robotPose.getTranslation()).getAngle();

        return Optional.of(angleToTag.minus(robotPose.getRotation()));
    }
}