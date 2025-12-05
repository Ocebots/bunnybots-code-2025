// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.TunerConstants;
import frc.robot.config.VisionConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  @Logged private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    // loop continuously runs as long as the robot is active
    CommandScheduler.getInstance().run();
    m_robotContainer.m_odometry.update(
        new Rotation2d(RobotContainer.pigeon2.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          drivetrain.getModule(0).getPosition(true),
          drivetrain.getModule(1).getPosition(true),
          drivetrain.getModule(2).getPosition(true),
          drivetrain.getModule(3).getPosition(true)
        });

        

        Command selectedAuto = m_robotContainer.getAutonomousCommand();
        if (selectedAuto != null) {
          SmartDashboard.putString("Selected Auto", selectedAuto.getName());
        } else {
            SmartDashboard.putString("Selected Auto", "None");
        }

    // Updates the stored reference pose for use when using the CLOSEST_TO_REFERENCE_POSE_STRATEGY
    // (not in use)
    VisionConfig.photonPoseEstimatorLeft.setReferencePose(
        m_robotContainer.m_odometry.getEstimatedPosition());
    VisionConfig.photonPoseEstimatorRight.setReferencePose(
        m_robotContainer.m_odometry.getEstimatedPosition());

    // Puts the pose data from one camera into a list
    List<PhotonPipelineResult> results = Vision.leftCameraApril.getAllUnreadResults();

    // If there is pose data from the cameras, get the latest estimated pose and update the 'vision'
    // photon pose estimator
    // If there is no multi tag result and the distance from the camera to the target is greater
    // than
    // 4 meters, return
    // Otherwise, add the latest vision pose estimate to a filter with the odometry pose estimate
    // and set
    // the guessed pose from that to the current pose
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      VisionConfig.photonPoseEstimatorLeft
          .update(result)
          .ifPresent(
              (pose) -> {
                if (result.multitagResult.isEmpty()
                    && result.targets.get(0).bestCameraToTarget.getTranslation().getNorm() > 4) {
                  return;
                }
                m_robotContainer.m_odometry.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds);
              });
    }

    results = Vision.rightCameraApril.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      VisionConfig.photonPoseEstimatorRight
          .update(result)
          .ifPresent(
              (pose) -> {
                if (result.multitagResult.isEmpty()
                    && result.targets.get(0).bestCameraToTarget.getTranslation().getNorm() > 4) {
                  return;
                }
                m_robotContainer.m_odometry.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds);
              });
    }
  }

  @Override
  public void disabledInit() {
    // runs once robot is disabled
  }

  @Override
  public void disabledPeriodic() {
    // robot is disabled but still running
  }

  @Override
  public void disabledExit() {
    // likely will never be used
  }

  @Override
  public void autonomousInit() {
    RobotContainer.zeroPigeon();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // nope screw that that's this one
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
