package frc.robot.config;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

public class VisionConfig {
  public static final String CAMERA_NAME = "leftAprilTag"; // left
  public static final String SECOND_CAMERA_NAME = "rightAprilTag"; // right
  public static final String THIRD_CAMERA_NAME = "driverVision"; // rear cam
  public static final ArrayList<AprilTag> APRIL_TAG_LIST =
      new ArrayList<AprilTag>(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      Units.inchesToMeters(72),
                      Units.inchesToMeters(320),
                      Units.inchesToMeters(14),
                      new Rotation3d(0, 0, 270))),
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(576),
                      Units.inchesToMeters(320),
                      Units.inchesToMeters(14),
                      new Rotation3d(0, 0, 270))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(4),
                      Units.inchesToMeters(270),
                      Units.inchesToMeters(14),
                      new Rotation3d(0, 0, 0))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(644),
                      Units.inchesToMeters(270),
                      Units.inchesToMeters(14),
                      new Rotation3d(0, 0, 180))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(4),
                      Units.inchesToMeters(196.125),
                      Units.inchesToMeters(46),
                      new Rotation3d(0, 0, 0))),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(644),
                      Units.inchesToMeters(196.125),
                      Units.inchesToMeters(46),
                      new Rotation3d(0, 0, 180))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(4),
                      Units.inchesToMeters(20.5),
                      Units.inchesToMeters(46),
                      new Rotation3d(0, 0, 0))),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(644),
                      Units.inchesToMeters(20.5),
                      Units.inchesToMeters(46),
                      new Rotation3d(0, 0, 180)))));

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      new AprilTagFieldLayout(APRIL_TAG_LIST, Units.inchesToMeters(648), Units.inchesToMeters(324));
  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final Transform3d LEFT_CAMERA_POSITION =
      new Transform3d(Units.inchesToMeters(9.25), Units.inchesToMeters(10.5), Units.inchesToMeters(7.5), new Rotation3d(0.0, Units.degreesToRadians(30), 0.0));
  public static final Transform3d RIGHT_CAMERA_POSITION =
          new Transform3d(Units.inchesToMeters(9.25), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.5), new Rotation3d(0.0, Units.degreesToRadians(30), 0.0));
  public static final Transform3d REAR_CAMERA_POSITION =
      new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  Optional<EstimatedRobotPose> visionEst = Optional.empty();
  public static PhotonPoseEstimator photonPoseEstimatorLeft =
      new PhotonPoseEstimator(FIELD_LAYOUT, STRATEGY, LEFT_CAMERA_POSITION);
  public static PhotonPoseEstimator photonPoseEstimatorRight =
      new PhotonPoseEstimator(FIELD_LAYOUT, STRATEGY, RIGHT_CAMERA_POSITION);
}
