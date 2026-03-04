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

    private boolean allowVisionFusion = false;

    // Four cameras
    private final PhotonCamera frontRightCam     = new PhotonCamera("frontRightCam");// RENAME IN PHOTON!!!!!!
    private final PhotonCamera frontLeftCam  = new PhotonCamera("frontLeftCam"); // RENAME IN PHOTON!!!!!!
  //  private final PhotonCamera leftFrontCam = new PhotonCamera("leftFrontCam");
  //  private final PhotonCamera rightCam     = new PhotonCamera("rightCam");

    // Camera → Robot transforms
    private static final Transform3d kRobotToFrontRightCam = new Transform3d(
        0.3429,  -0.2667, 0.758063, new Rotation3d(0, Math.toRadians(13), Math.toRadians(0))
    );
     private static final Transform3d kRobotToFrontLeftCam = new Transform3d(
         0.2413, 0.3175, 0.52705, new Rotation3d(Math.toRadians(90), Math.toRadians(0), Math.toRadians(0))
     );
    //  private static final Transform3d kRobotToLeftFrontCam = new Transform3d(
    //      0.3175,  0.3175, 0.45085, new Rotation3d(0, Math.toRadians(15), Math.toRadians(45))
    //  );
    //  private static final Transform3d kRobotToRightCam = new Transform3d(
    //     -0.0762, -0.3429, 0.5242052, new Rotation3d(0, 0, Math.toRadians(-90))
    //  );

    private final CommandSwerveDrivetrain swerve;
    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estFrontRight;
    private final PhotonPoseEstimator estFrontLeft;
    //private final PhotonPoseEstimator estLeftFront;
    //private final PhotonPoseEstimator estRight;

    private final Field2d fieldVisualizer = new Field2d();

    private final Translation2d shooterOffset =
    new Translation2d(0.2921, -0.2286);

    public Vision(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout layout) {
        this.swerve = drivetrain;
        this.fieldLayout = layout;

        // IMPORTANT: Pass the PhotonCamera instance into the PhotonPoseEstimator
        estFrontRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToFrontRightCam);
        estFrontLeft  = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToFrontLeftCam);
     //   estLeftFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToLeftFrontCam);
     //   estRight     = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,     kRobotToRightCam);

        SmartDashboard.putData("Vision Field", fieldVisualizer);
    }

    @Override
    public void periodic() {

        Pose2d odomPose = swerve.getState().Pose;
        fieldVisualizer.setRobotPose(odomPose);

        SmartDashboard.putBoolean("front right has targets?", frontRightCam.getLatestResult().hasTargets());
        SmartDashboard.putBoolean("front left has targets?", frontLeftCam.getLatestResult().hasTargets());
       // SmartDashboard.putBoolean("leftFrontCam has targets?", leftFrontCam.getLatestResult().hasTargets());
     //   SmartDashboard.putBoolean("rightCam has targets?", rightCam.getLatestResult().hasTargets());
        SmartDashboard.putNumber("Robot Heading", odomPose.getRotation().getDegrees());
        getDirectRotationErrorShooterToTag(10).ifPresent(rot -> SmartDashboard.putNumber("Angle To Hub from shooter", rot.getDegrees()));
        getDirectRotationErrorToTag(10).ifPresent(rot -> SmartDashboard.putNumber("Angle To Hub from robot center", rot.getDegrees()));
        SmartDashboard.putNumber("gyro Heading", swerve.getState().Pose.getRotation().getDegrees());

        getDirectDistanceToTag(10).ifPresent(distance -> SmartDashboard.putNumber("Distance pure vision not from Pose", distance)); 


        estFrontRight.setReferencePose(odomPose);
        estFrontLeft.setReferencePose(odomPose);
      //  estLeftFront.setReferencePose(odomPose);
      //  estRight.setReferencePose(odomPose);

        Optional<EstimatedRobotPose> poseFrontRight     = estFrontRight.update(frontRightCam.getLatestResult());
        Optional<EstimatedRobotPose> poseFrontLeft  = estFrontLeft.update(frontLeftCam.getLatestResult());
     //   Optional<EstimatedRobotPose> poseLeftFront = estLeftFront.update(leftFrontCam.getLatestResult());
     //   Optional<EstimatedRobotPose> poseRight     = estRight.update(rightCam.getLatestResult());

        List<EstimatedRobotPose> poses = new ArrayList<>();
        List<Integer> weights = new ArrayList<>();
        List<PhotonCamera> cams = new ArrayList<>();

        addPoseIfValid(poseFrontRight,     frontRightCam,     poses, weights, cams);
        addPoseIfValid(poseFrontLeft,  frontLeftCam,  poses, weights, cams);
     //   addPoseIfValid(poseLeftFront, leftFrontCam, poses, weights, cams);
     //   addPoseIfValid(poseRight,     rightCam,     poses, weights, cams);

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

        if (allowVisionFusion) {
        swerve.addVisionMeasurement(
            fused,
            timestamp / totalWeight,
            stdDevs
                );
        }

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
       if (poseOpt.isPresent()) {
            Pose2d visionPose = poseOpt.get().estimatedPose.toPose2d();
            SmartDashboard.putNumber(
            cam.getName() + " vision heading",
        visionPose.getRotation().getDegrees()
    );
}
    }

    // --- Helper function: distance to a specific tag ID ---
    public Optional<Double> getDirectDistanceToTag(int tagId) {
        List<PhotonCamera> cameras = List.of(frontRightCam, frontLeftCam); 

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

    public Optional<Rotation2d> getDirectRotationErrorToTag(int tagId) {
    List<PhotonCamera> cameras = List.of(frontRightCam, frontLeftCam);
    // Corresponding yaw offsets for the cameras from your Transform3d constants
    // frontRightCam yaw is 0, frontLeftCam yaw is 0 (based on your code)
    double[] camYawOffsets = {0.0, 0.0}; 

    for (int i = 0; i < cameras.size(); i++) {
        var result = cameras.get(i).getLatestResult();
        if (!result.hasTargets()) continue;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagId) {
                // target.getYaw() is the angle from the CAMERA center to the tag.
                // We add the camera's mounting yaw to get the angle relative to the ROBOT center.
                double robotRelativeErrorDeg = target.getYaw() + camYawOffsets[i];
                
                // Return as a Rotation2d (Note: Photon Yaw is usually CCW-positive)
                return Optional.of(Rotation2d.fromDegrees(-robotRelativeErrorDeg));
            }
        }
    }
    return Optional.empty();
}

public Optional<Rotation2d> getDirectRotationErrorShooterToTag(int tagId) {
    return getDirectTranslationToTag(tagId).map(robotToTag -> {
        // 1. Get Vector from Shooter to Tag in Robot Space
        // We subtract where the shooter is relative to center
        Translation2d shooterToTag = robotToTag.minus(shooterOffset);

        // 2. Calculate the angle of that vector
        // This is the angle the ROBOT must be at for the SHOOTER to face the tag
        return new Rotation2d(shooterToTag.getX(), shooterToTag.getY());
    });
}

public Optional<Translation2d> getDirectTranslationToTag(int tagId) {
    // We'll check both cameras, but prioritize FrontRight for this example
    List<PhotonCamera> cameras = List.of(frontRightCam, frontLeftCam);
    List<Transform3d> robotToCams = List.of(kRobotToFrontRightCam, kRobotToFrontLeftCam);

    for (int i = 0; i < cameras.size(); i++) {
        var result = cameras.get(i).getLatestResult();
        if (!result.hasTargets()) continue;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagId) {
                // 1. Get the transform from the CAMERA to the TAG
                Transform3d camToTag = target.getBestCameraToTarget();

                // 2. Transform that into ROBOT coordinates
                // RobotToTag = RobotToCamera * CameraToTag
                Transform3d robotToTag = robotToCams.get(i).plus(camToTag);

                // 3. Return the 2D component (X/Y distance from robot center)
                return Optional.of(new Translation2d(
                    robotToTag.getX(), 
                    robotToTag.getY()
                ));
            }
        }
    }
    return Optional.empty();
}

public Optional<Translation2d> getDirectTranslationShooterToTag(int tagId) {
    List<PhotonCamera> cameras = List.of(frontRightCam, frontLeftCam);
    List<Transform3d> robotToCams = List.of(kRobotToFrontRightCam, kRobotToFrontLeftCam);

    for (int i = 0; i < cameras.size(); i++) {
        var result = cameras.get(i).getLatestResult();
        if (!result.hasTargets()) continue;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagId) {
                // 1. Tag relative to Camera
                Transform3d camToTag = target.getBestCameraToTarget();
                
                // 2. Tag relative to Robot Center
                Transform3d robotToTag = robotToCams.get(i).plus(camToTag);
                Translation2d robotToTagTranslation = robotToTag.getTranslation().toTranslation2d();

                // 3. Subtract Shooter Offset (Robot Center to Shooter)
                // Result: Vector from Shooter to Tag
                return Optional.of(robotToTagTranslation.minus(shooterOffset));
            }
        }
    }
    return Optional.empty();
}



public Optional<Rotation2d> getRotationErrorRobotToTagFromPose(int tagId) {

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

    public Optional<Rotation2d> getRotationErrorShooterToTagFromPose(int tagId) {

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





    public void setVisionEnabled(boolean enabled) {
            allowVisionFusion = enabled;
        }
}