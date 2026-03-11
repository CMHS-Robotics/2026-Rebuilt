package frc.robot.tools;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class FieldUtil {

    private static final double HUB_OFFSET = 0.6;

    /** Returns the translation of the tag */
    public static Optional<Translation2d> getTagTranslation(AprilTagFieldLayout layout,int tagId) {
        return layout.getTagPose(tagId)
            .map(tagPose -> tagPose.toPose2d().getTranslation());
    }

    /** Returns the hub center by offsetting back from the tag */
    public static Optional<Translation2d> getHubTranslation(AprilTagFieldLayout layout,int tagId) {
        return layout.getTagPose(tagId).map(tagPose3d -> {

            Pose2d tagPose = tagPose3d.toPose2d();

            Translation2d hubOffset =
                new Translation2d(-HUB_OFFSET, 0)
                    .rotateBy(tagPose.getRotation());

            return tagPose.getTranslation().plus(hubOffset);
        });
    }

    /** Returns the hub pose (useful for aiming) */
    public static Optional<Pose2d> getHubPose(AprilTagFieldLayout layout,int tagId) {
        return layout.getTagPose(tagId).map(tagPose3d -> {

            Pose2d tagPose = tagPose3d.toPose2d();

            Translation2d hubOffset =
                new Translation2d(-HUB_OFFSET, 0)
                    .rotateBy(tagPose.getRotation());

            return new Pose2d(
                tagPose.getTranslation().plus(hubOffset),
                tagPose.getRotation()
            );
        });
    }

}