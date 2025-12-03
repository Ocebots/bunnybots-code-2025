// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.config.CANMappings;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType
                  .OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

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
    final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -controller.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -controller.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    controller.leftTrigger().onTrue(superstructure.toggleCloseHigh());
    // controller.rightTrigger().onTrue(superstructure.action());
    // controller.leftBumper().onTrue(superstructure.toggleFarHigh());
    // controller.rightBumper().onTrue(superstructure.toggleLowScore());
    // controller.rightStick().onTrue(superstructure.toggleIntake());

    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(Units.inchesToMeters(59.0), 0.18);
    map.put(Units.inchesToMeters(76.5), 0.155);
    map.put(Units.inchesToMeters(96.5), 0.142);
    map.put(Units.inchesToMeters(125.5), 0.13);
    map.put(Units.inchesToMeters(169.5), 0.12);
    map.put(Units.inchesToMeters(210.5), 0.118);

    // Pivot to angle based off distance
    controller
        .povUp()
        .whileTrue(
            (Commands.run(
                () ->
                    pivot.setPivotAngleRot(
                        map.get(
                            drivetrain
                                .getState()
                                .Pose
                                .getTranslation()
                                .getDistance(pivot.getCosmicConverterTranslation(false)))))));

    // Pivot default command
    pivot.setDefaultCommand(Commands.run(() -> pivot.setPivotAngleRot(0.0)));
    // Zero pivot
    controller.povLeft().onTrue(Commands.runOnce(() -> pivot.zeroPivot()));
    // Shoot
    controller
        .rightTrigger()
        .whileTrue(
            Commands.run(() -> shooter.shoot(1))
                .alongWith(Commands.run(() -> intake.runKicker(-0.3))));
    // Shooter default command
    shooter.setDefaultCommand(Commands.run(() -> shooter.stopShooter()));

    // Intake
    controller.leftTrigger().whileTrue(Commands.run(() -> intake.intake()));
    intake.setDefaultCommand(Commands.run(() -> intake.stopIntake()));

    // Drivetrain default command
    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.setControl(
                    (new SwerveRequest.FieldCentric()
                        .withVelocityX(controller.getLeftY())
                        .withVelocityY(controller.getLeftX())
                        .withRotationalRate(
                            controller.getRightX()))))); // Drive counterclockwise with negative X
    // auto align with inner cosmic converter
    controller
        .rightBumper()
        .toggleOnTrue(
            pivot
                .getCosmicConverter(true)
                .alongWith(
                    Commands.run(
                        (() ->
                            pivot.setPivotAngleRot(
                                map.get(
                                    drivetrain
                                        .getState()
                                        .Pose
                                        .getTranslation()
                                        .getDistance(
                                            pivot.getCosmicConverterTranslation(true))))))));

    // auto align with outer cosmic converter
    controller
        .rightBumper()
        .toggleOnTrue(
            pivot
                .getCosmicConverter(false)
                .alongWith(
                    Commands.run(
                        (() ->
                            pivot.setPivotAngleRot(
                                map.get(
                                    drivetrain
                                        .getState()
                                        .Pose
                                        .getTranslation()
                                        .getDistance(
                                            pivot.getCosmicConverterTranslation(false))))))));
    // Outtake
    controller.povDown().toggleOnTrue(Commands.run(() -> intake.outtake()));
    // Low score
    controller.povRight().toggleOnTrue(Commands.run(() -> pivot.setPivotAngleRot(0.0)));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
