package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.ml.SVM;
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
    private final PhotonCamera frontCam     = new PhotonCamera("frontCam");
    private final PhotonCamera leftBackCam  = new PhotonCamera("leftBackCam");
    private final PhotonCamera leftFrontCam = new PhotonCamera("leftFrontCam");
    private final PhotonCamera rightCam     = new PhotonCamera("rightCam");

    // Camera → Robot transforms
    private static final Transform3d kRobotToFrontCam = new Transform3d(
        0.3429,  -0.2667, 0.758063, new Rotation3d(0, Math.toRadians(13), Math.toRadians(0))
    );
     private static final Transform3d kRobotToLeftBackCam = new Transform3d(
         0.2413, 0.3175, 0.52705, new Rotation3d(0, Math.toRadians(15), Math.toRadians(135))
     );
     private static final Transform3d kRobotToLeftFrontCam = new Transform3d(
         0.3175,  0.3175, 0.45085, new Rotation3d(0, Math.toRadians(15), Math.toRadians(45))
     );
     private static final Transform3d kRobotToRightCam = new Transform3d(
        -0.0762, -0.3429, 0.5242052, new Rotation3d(0, 0, Math.toRadians(-90))
     );

    private final CommandSwerveDrivetrain swerve;
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estFront;
    private final PhotonPoseEstimator estLeftBack;
    private final PhotonPoseEstimator estLeftFront;
    private final PhotonPoseEstimator estRight;

    private final Field2d fieldVisualizer = new Field2d();

    private final Translation2d shooterOffset =
    new Translation2d(0.2921, -0.2286);

    public Vision(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout layout) {
        this.swerve = drivetrain;
        this.fieldLayout = layout;

        // IMPORTANT: Pass the PhotonCamera instance into the PhotonPoseEstimator
        estFront     = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, kRobotToFrontCam);
        estLeftBack  = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, kRobotToLeftBackCam);
        estLeftFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, kRobotToLeftFrontCam);
        estRight     = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY,     kRobotToRightCam);

        SmartDashboard.putData("Vision Field", fieldVisualizer);
    }

    @Override
    public void periodic() {

        Pose2d odomPose = swerve.getState().Pose;
        fieldVisualizer.setRobotPose(odomPose);

        SmartDashboard.putBoolean("frontCam has targets?", frontCam.getLatestResult().hasTargets());
        SmartDashboard.putBoolean("leftBackCam has targets?", leftBackCam.getLatestResult().hasTargets());
        SmartDashboard.putBoolean("leftFrontCam has targets?", leftFrontCam.getLatestResult().hasTargets());
        SmartDashboard.putBoolean("rightCam has targets?", rightCam.getLatestResult().hasTargets());
        SmartDashboard.putNumber("Robot Heading", odomPose.getRotation().getDegrees());
        getRotationErrorShooterToTag(10).ifPresent(rot -> SmartDashboard.putNumber("Angle To Hub from shooter", rot.getDegrees()));

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

        SmartDashboard.putNumber("total vision poses", poses.size());
        
        if (poses.isEmpty()) {
        SmartDashboard.putBoolean("V fused", false);
         return;
        }

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

            if (pose2d.getTranslation().getDistance(odomPose.getTranslation()) > 10.0)
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
        double thetaStd = .05; // 0.2 + 0.1 * avgDist; //changed this to be less restrictive

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

        fieldVisualizer.setRobotPose(swerve.getState().Pose);
        SmartDashboard.putBoolean("V fused", true);

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
    
        Pose2d robotPose = swerve.getState().Pose;
    
        Translation2d tagTranslation =
            tagPoseOpt.get().toPose2d().getTranslation();
    
        // Rotate shooter offset into field frame
      //  Translation2d shooterFieldOffset = shooterOffset.rotateBy(robotPose.getRotation());
    
        // Shooter world position
        // Translation2d shooterWorldPos = robotPose.getTranslation().plus(shooterFieldOffset);
    
      //  return Optional.of(
      //      shooterWorldPos.getDistance(tagTranslation)
      //  );

      Translation2d robotTranslation = robotPose.getTranslation();
      return Optional.of(robotTranslation.getDistance(tagTranslation));
    }

    public Optional<Translation2d> translationShooterToTagFromPose(int tagId) {

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();
    
        Pose2d robotPose = swerve.getState().Pose;
    
        Translation2d tagTranslation =
            tagPoseOpt.get().toPose2d().getTranslation();
    
        Translation2d shooterFieldOffset =
            shooterOffset.rotateBy(robotPose.getRotation());
    
        Translation2d shooterWorldPos =
            robotPose.getTranslation().plus(shooterFieldOffset);
    
        return Optional.of(
            tagTranslation.minus(shooterWorldPos)
        );
    }

    public Optional<Translation2d> translationToTagFromPose(int tagId) {

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();
    
        Pose2d robotPose = swerve.getState().Pose;
    
        Translation2d tagTranslation =
            tagPoseOpt.get().toPose2d().getTranslation();
    
        Translation2d robotTranslation = robotPose.getTranslation();
            
    
        return Optional.of(
            tagTranslation.minus(robotTranslation)
        );
    }

    public Optional<Rotation2d> getRotationErrorShooterToTag(int tagId) {

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
    
        Pose2d robotPose = swerve.getState().Pose;
    
        // Hub/tag world position
        Translation2d tagPos =
            tagPoseOpt.get().toPose2d().getTranslation();
    
        // ---- Compute Shooter World Position ----
    
        // Rotate shooter offset into field frame
        Translation2d shooterFieldOffset =
            shooterOffset.rotateBy(robotPose.getRotation());
    
        // Shooter world position
        Translation2d shooterWorldPos =
            robotPose.getTranslation().plus(shooterFieldOffset);
    
        // ---- Compute Vector From Shooter To Hub ----
    
        Translation2d vectorToHub =
            tagPos.minus(shooterWorldPos);
    
        Rotation2d angleToHub =
            vectorToHub.getAngle();
    
        // Rotation error = desired heading - current heading
        Rotation2d rotError = angleToHub.minus(robotPose.getRotation());
    
        return Optional.of(rotError);
    }

    public Optional<Rotation2d> getRotationErrorRobotToTag(int tagId) {

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
    
        Pose2d robotPose = swerve.getState().Pose;

        Translation2d  robotTranslation = robotPose.getTranslation();
    
        // Hub/tag world position
        Translation2d tagPos = tagPoseOpt.get().toPose2d().getTranslation();
    
    
        // ---- Compute Vector From Robot To Hub ----
    
        Translation2d vectorToHub = tagPos.minus(robotTranslation);
    
        Rotation2d angleToHub =
            vectorToHub.getAngle();
    
        // Rotation error = desired heading - current heading
        Rotation2d rotError = angleToHub.minus(robotPose.getRotation());
    
        return Optional.of(rotError);
    }
}