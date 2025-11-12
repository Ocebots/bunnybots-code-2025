package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
  public static final PhotonCamera mainCamera = new PhotonCamera(VisionConfig.MAIN_CAMERA_NAME); // Might be used for aprilTags? but i dunno

  public static final PhotonCamera aprilLeftCamera =
      new PhotonCamera(VisionConfig.APRIL_LEFT_CAMERA_NAME);
  public static final PhotonCamera aprilRightCamera =
      new PhotonCamera(VisionConfig.APRIL_RIGHT_CAMERA_NAME);
}
