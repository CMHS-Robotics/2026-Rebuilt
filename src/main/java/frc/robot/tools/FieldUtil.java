package frc.robot.tools;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldUtil {
    public static Translation2d getHubPosition(AprilTagFieldLayout layout, int tagID) {

    Pose3d tagPose = layout.getTagPose(tagID).get();

    Pose2d tagPose2d = tagPose.toPose2d();

    Translation2d hubPosition =
        tagPose2d.getTranslation().plus(
            new Translation2d(-0.6, 0).rotateBy(tagPose2d.getRotation())
        );

    return hubPosition;
}


}
