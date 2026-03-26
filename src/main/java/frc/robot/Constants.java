// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
    public static class Vision {
        public static final String frontCam = "frontCam";
        public static final String frontLeftCam = "frontLeftCam";
        public static final String backCam = "backCam";
        public static final String backRightCam = "backRightCam";
        public static final String backLeftCam = "backLeftCam";
        
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        public static final Transform3d kRobotToFrontCam = new Transform3d(
        0.3556,  -0.06985, 0.4699, new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
        );

        public static final Transform3d kRobotToBackCam = new Transform3d(
        -0.16383, -0.23495, 0.45085, new Rotation3d(Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(180))
        );

        public static final Transform3d kRobotToBackRightCam = new Transform3d(
        -0.20955, -0.33655, 0.45085, new Rotation3d(Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(135))
        );

        public static final Transform3d kRobotToBackLeftCam = new Transform3d(
        -0.320675, -0.127, 0.45085, new Rotation3d(Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(225))
        );


        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)

        // Lower values = MORE trust in the camera
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2); 
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);
    //    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.5, .5, 9999999);
    //    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 9999999);
    }
}
