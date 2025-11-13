package frc.robot.config;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.photonvision.PhotonPoseEstimator;

public final class VisionConfig {
  public static final String MAIN_CAMERA_NAME = "mainCamera";
  public static final String APRIL_LEFT_CAMERA_NAME = "aprilLeftCamera";
  public static final String APRIL_RIGHT_CAMERA_NAME = "aprilRightCamera";

  public static final double FIELD_LENGTH = Units.inchesToMeters(57);
  public static final double FIELD_WIDTH = Units.inchesToMeters(27);

  public static final List<AprilTag>
      aprilTags = // Using a List.of, as it is immutable. Kinda redundant cus of the final keyword,
          // but its probs fine 👍
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(72.0),
                          Units.inchesToMeters(320.0),
                          Units.inchesToMeters(14.0)),
                      new Rotation3d(0.0, 0.0, 270.0))),
              new AprilTag(
                  2,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(576.0),
                          Units.inchesToMeters(320.0),
                          Units.inchesToMeters(14.0)),
                      new Rotation3d(0.0, 0.0, 270.0))),
              new AprilTag(
                  3,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(4.0),
                          Units.inchesToMeters(270.0),
                          Units.inchesToMeters(14.0)),
                      new Rotation3d(0.0, 0.0, 0.0))),
              new AprilTag(
                  4,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(644.0),
                          Units.inchesToMeters(270.0),
                          Units.inchesToMeters(14.0)),
                      new Rotation3d(0.0, 0.0, 180.0))),
              new AprilTag(
                  5,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(4.0),
                          Units.inchesToMeters(196.125),
                          Units.inchesToMeters(46.0)),
                      new Rotation3d(0.0, 0.0, 0.0))),
              new AprilTag(
                  6,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(644.0),
                          Units.inchesToMeters(196.125),
                          Units.inchesToMeters(46.0)),
                      new Rotation3d(0.0, 0.0, 180.0))),
              new AprilTag(
                  7,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(4.0),
                          Units.inchesToMeters(20.5),
                          Units.inchesToMeters(46.0)),
                      new Rotation3d(0.0, 0.0, 0.0))),
              new AprilTag(
                  8,
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(644.0),
                          Units.inchesToMeters(20.5),
                          Units.inchesToMeters(46.0)),
                      new Rotation3d(0.0, 0.0, 180.0))));

  public static final AprilTagFieldLayout bunnyBotsFieldLayout =
      new AprilTagFieldLayout(aprilTags, FIELD_LENGTH, FIELD_WIDTH);

  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  /*
  //These are the previous year's camera locations.
  public static final Transform3d LEFT_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(12.25),
          Units.inchesToMeters(10.75),
          0.0,
          new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d RIGHT_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(12.25),
          Units.inchesToMeters(-8.75),
          0.0,
          new Rotation3d(0.0, 0.0, 0.0));
   */
}
