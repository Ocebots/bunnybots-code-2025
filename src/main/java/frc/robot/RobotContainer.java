// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.CANMappings;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Intake intake = new Intake();
  private Pivot pivot = new Pivot();
  private Shooter shooter = new Shooter();
  private final Superstructure superstructure =
      new Superstructure(intake, pivot, shooter, drivetrain);
  public static Pigeon2 pigeon2 = new Pigeon2(CANMappings.PIGEON_CAN_ID);
  Translation2d m_frontLeftLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(10.875));
  Translation2d m_frontRightLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(-10.875));
  Translation2d m_backLeftLocation =
      new Translation2d(Units.inchesToMeters(-10.875), Units.inchesToMeters(10.875));
  Translation2d m_backRightLocation =
      new Translation2d(Units.inchesToMeters(-10.855), Units.inchesToMeters(-10.875));
  SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(
              m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation),
          pigeon2.getRotation2d(),
          new SwerveModulePosition[] {
            drivetrain.getModule(0).getPosition(true),
            drivetrain.getModule(1).getPosition(true),
            drivetrain.getModule(2).getPosition(true),
            drivetrain.getModule(3).getPosition(true)
          },
          new Pose2d(0.0, 0.0, new Rotation2d()));
  static Field2d m_field = new Field2d();

  // links xbox controller to controls
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.leftTrigger().onTrue(superstructure.toggleCloseHigh());
    // controller.rightTrigger().onTrue(superstructure.action());
    // controller.leftBumper().onTrue(superstructure.toggleFarHigh());
    // controller.rightBumper().onTrue(superstructure.toggleLowScore());
    // controller.rightStick().onTrue(superstructure.toggleIntake());

    // Pivot to angle
    controller.povUp().whileTrue((Commands.run(() -> pivot.setPivotAngleRot(0.17))));
    // Zero pivot
    controller.povLeft().onTrue(Commands.runOnce(() -> pivot.zeroPivot()));
    // Intake and shoot
    controller
        .rightTrigger()
        .whileTrue(
            Commands.run(() -> shooter.shoot(1))
                .alongWith(Commands.run(() -> intake.intake())));
    // Stop everything
    controller
        .a()
        .onTrue(Commands.run(() -> intake.stopIntake()).andThen(() -> shooter.stopShooter()));
    // Intake
    controller.leftTrigger().whileTrue(Commands.run(() -> intake.intake()));
    // Drive
    drivetrain.setControl(
        (new SwerveRequest.FieldCentric()
            .withVelocityX(controller.getLeftY())
            .withVelocityY(controller.getLeftX())
            .withRotationalRate(controller.getRightX()))); // Drive counterclockwise with negative X



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
