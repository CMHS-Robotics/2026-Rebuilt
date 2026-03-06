package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

public class PoseEstimation {
    private final CommandSwerveDrivetrain swerve;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;
    private Matrix<N3, N1> curStdDevs;
    private final Field2d fieldVisualizer = new Field2d();

    private final Translation2d shooterOffset = new Translation2d(0.2921, -0.2286);

    public PoseEstimation(CommandSwerveDrivetrain swerve) {
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

    public Optional<Rotation2d> getRotationErrorRobotToTagFromPose(int tagId, Pose2d robotPose) {
        return kTagLayout.getTagPose(tagId).map(tagPose -> {
            Translation2d vectorToHub = tagPose.toPose2d().getTranslation().minus(robotPose.getTranslation());
            return vectorToHub.getAngle().minus(robotPose.getRotation());
        });
    }

    

    public Optional<Double> distanceToTagFromPose(int tagId, Pose2d robotPose) {
        return kTagLayout.getTagPose(tagId).map(tagPose -> 
            robotPose.getTranslation().getDistance(tagPose.toPose2d().getTranslation())
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


    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
