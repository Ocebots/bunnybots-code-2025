package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pivot;

public class DrivetrainCommand extends Command {
  public static enum Position {
    OUTER_COSMIC_CONVERTER,
    INNER_COSMIC_CONVERTER
  }

  private DrivetrainCommand.Position state;
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d robotPose;
  private Translation2d diff;
  private Rotation2d targetRotation;
  private Translation2d cosmicConverter;

  // Initialize the facing angle request
  private final SwerveRequest.FieldCentricFacingAngle m_faceAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // Or OpenLoopDutyCycle

  public DrivetrainCommand(CommandSwerveDrivetrain drivetrain, DrivetrainCommand.Position state) {
    this.drivetrain = drivetrain;
    this.state = state;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    switch (state) {
      case OUTER_COSMIC_CONVERTER:
        cosmicConverter = Pivot.getLocation(0);
        drivetrain.setControl(
            m_faceAngle
                .withVelocityX(0)
                .withVelocityY(0)
                // Set the desired direction in Radians
                .withTargetDirection(
                    new Rotation2d(
                        Math.atan2(
                            cosmicConverter.getX() - drivetrain.getState().Pose.getX(),
                            cosmicConverter.getY() - drivetrain.getState().Pose.getY()))));
        break;

      case INNER_COSMIC_CONVERTER:
        cosmicConverter = Pivot.getLocation(1);
        drivetrain.setControl(
            m_faceAngle
                .withVelocityX(0)
                .withVelocityY(0)
                // Set the desired direction in Radians
                .withTargetDirection(
                    new Rotation2d(
                        Math.atan2(
                            cosmicConverter.getX() - drivetrain.getState().Pose.getX(),
                            cosmicConverter.getY() - drivetrain.getState().Pose.getY()))));
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
