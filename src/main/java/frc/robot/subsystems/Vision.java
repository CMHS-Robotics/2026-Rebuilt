package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.estimation.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// public class Vision extends SubsystemBase {

//      private PhotonCamera camLeft = new PhotonCamera("FrontLeftCamera");
//      private PhotonCamera camRight = new PhotonCamera("FrontRightCamera");

//     // need to be updated for new bot
//     private static final Transform3d kRobotToFrontLeftCamera =
//         new Transform3d(
//             0.3625,  0.22, 0.12,
//             new Rotation3d(0, 0, Math.toRadians(15))
//         );

//     private static final Transform3d kRobotToFrontRightCamera =
//         new Transform3d(
//             0.3625, -0.22, 0.12,
//             new Rotation3d(0, 0, Math.toRadians(-15))
//         );

//     private static final Transform3d kRobotToBackLeftCamera =
//         new Transform3d(
//             0.3625, -0.22, 0.12,
//             new Rotation3d(0, 0, Math.toRadians(-15))
//     );

//     private static final Transform3d kRobotToBackRightCamera =
//         new Transform3d(
//             0.3625, -0.22, 0.12,
//             new Rotation3d(0, 0, Math.toRadians(-15))
//     );



//     private final CommandSwerveDrivetrain swerve;

//     // private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
//     // This is the actual feild ^ but were using a custom test feild 
//     private final AprilTagFieldLayout fieldLayout; // this is the test feild that I made
    
//     private final PhotonPoseEstimator leftEstimator;
//     private final PhotonPoseEstimator rightEstimator;

//     private final Field2d fieldVisualizer = new Field2d();


    

//     private Pose2d latestFieldPose = new Pose2d();
//     private int selectedDestinationTag = -1;

//     public Vision(CommandSwerveDrivetrain s) {
//     this.swerve = s;
//    // this.driver = d;

//     // --- START: CUSTOM TEST FIELD LAYOUT DEFINITION ---
//     List<AprilTag> testTags = new ArrayList<>();

//     // Tag 1: Placed at (X=1.0m, Y=0.0m, Z=0.5m), facing 0 degrees yaw
//     testTags.add(new AprilTag(
//         1, 
//         new Pose3d(
//             4.0, -1.0, 0.508, 
//             new Rotation3d(0, 0, Math.toRadians(0))
//         )
//     ));

//     //Tag 2: Placed at (X=1.0m, Y=1.0m, Z=0.5m), facing 0 degrees yaw
//     testTags.add(new AprilTag(
//         2, 
//         new Pose3d(
//             2.0, -4.0, 0.508, 
//             new Rotation3d(0, 0, Math.toRadians(90))
//         )
//     ));

//     testTags.add(new AprilTag(
//         3, 
//         new Pose3d(
//             4.0, -3.0, 0.508, 
//             new Rotation3d(0, 0, Math.toRadians(0))
//         )
//     ));
    
//     // Add any other tags we have printed (e.g., Tag 3, Tag 4)

//     // Create the test field layout (adjust field dimensions if needed)
//     this.fieldLayout = new AprilTagFieldLayout(testTags, 5.0, 5.0); // 5m x 5m test space

//     // --- END: CUSTOM TEST FIELD LAYOUT DEFINITION ---
//     frontLeftEstimator = new PhotonPoseEstimator( fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToFrontLeftCamera.inverse());

//     frontRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToFrontRightCamera.inverse());

//     backLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,kRobotToBackLeftCamera.inverse());

//     backRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,kRobotToBackRightCamera.inverse());

//     SmartDashboard.putData("Field", fieldVisualizer);
// }

//     public void setDestinationTag(int id) {
//         selectedDestinationTag = id;
//     }

//     public int getDestinationTag() {
//         return selectedDestinationTag;
//     }

//     // Finds the target we want to move toward.
//     private Optional<PhotonTrackedTarget> getChosenTarget() {

//         int destTag = selectedDestinationTag;

//         var left = camLeft.getLatestResult();
//         var right = camRight.getLatestResult();

//         // --- PRIORITY 1: Selected destination tag ---
//         if (destTag != -1) {
//             if (left.hasTargets()) {
//                 for (var t : left.getTargets()) {
//                     if (t.getFiducialId() == destTag)
//                         return Optional.of(t);
//                 }
//             }

//             if (right.hasTargets()) {
//                 for (var t : right.getTargets()) {
//                     if (t.getFiducialId() == destTag)
//                         return Optional.of(t);
//                 }
//             }
//         }

//         // --- PRIORITY 2: Best available target ---
//         if (left.hasTargets())
//             return Optional.of(left.getBestTarget());

//         if (right.hasTargets())
//             return Optional.of(right.getBestTarget());

//         return Optional.empty();
//     }

//     // *** Returns ROBOT → TAG transform ***
//     public Optional<Transform3d> getBestTransform() {

//         Optional<PhotonTrackedTarget> chosen = getChosenTarget();
//         if (chosen.isEmpty()) return Optional.empty();

//         PhotonTrackedTarget target = chosen.get();

//         // Which camera saw it?
//         boolean seenLeft =
//             camLeft.getLatestResult().getTargets().contains(target);

//         Transform3d cameraToTag = target.getBestCameraToTarget();
//         Transform3d robotToTag;

//         if (seenLeft) {
//             robotToTag = kFrontLeftCameraToRobot.plus(cameraToTag);
//         } else {
//             robotToTag = kFrontRightCameraToRobot.plus(cameraToTag);
//         }

//         return Optional.of(robotToTag);
//     }

//     public Pose2d getFieldPose() {
//         return latestFieldPose;
//     }

//     @Override
//     public void periodic() {

//         var leftResult = camLeft.getLatestResult();
//         var rightResult = camRight.getLatestResult();

//         Pose2d referencePose = swerve.getPose();

//         leftEstimator.setReferencePose(swerve.getState().Pose);
//         rightEstimator.setReferencePose(swerve.getState().Pose);

//         Optional<EstimatedRobotPose> leftPose = leftEstimator.update(leftResult);
//         Optional<EstimatedRobotPose> rightPose = rightEstimator.update(rightResult);
//         fieldVisualizer.setRobotPose(latestFieldPose);

//         if (leftPose.isPresent()) {
//             Pose2d pose2d = leftPose.get().estimatedPose.toPose2d();
//             swerve.addVisionMeasurement(pose2d, leftPose.get().timestampSeconds);
//             latestFieldPose = pose2d;
//         }
//         else if (rightPose.isPresent()) {
//             Pose2d pose2d = rightPose.get().estimatedPose.toPose2d();
//             swerve.addVisionMeasurement(pose2d, rightPose.get().timestampSeconds);
//             latestFieldPose = pose2d;
//         }

        

//         fieldVisualizer.setRobotPose(latestFieldPose);

//         SmartDashboard.putNumber("FieldPoseX", latestFieldPose.getX());
//         SmartDashboard.putNumber("FieldPoseY", latestFieldPose.getY());
//         SmartDashboard.putNumber("FieldPoseHeading", latestFieldPose.getRotation().getDegrees());
//         SmartDashboard.putNumber("DestinationTag", selectedDestinationTag);
//     }
//}

