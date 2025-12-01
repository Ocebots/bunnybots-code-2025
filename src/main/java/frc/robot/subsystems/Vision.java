package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.*;

public class Vision extends SubsystemBase {
  public static final PhotonCamera leftCameraApril = new PhotonCamera(VisionConfig.CAMERA_NAME);
  public static final PhotonCamera rightCameraApril =
      new PhotonCamera(VisionConfig.SECOND_CAMERA_NAME);
  public static final PhotonCamera rearCameraDriver =
      new PhotonCamera(VisionConfig.THIRD_CAMERA_NAME);
}
