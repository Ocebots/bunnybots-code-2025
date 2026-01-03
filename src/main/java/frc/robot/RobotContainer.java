// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          .withDeadband(MaxSpeed * 0.2)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType
                  .OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> autoChooser;
  private CommandXboxController controller = new CommandXboxController(0);
  public CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Intake intake = new Intake();
  private Pivot pivot = new Pivot();
  private Shooter shooter = new Shooter();
    private final SwerveRequest.FieldCentricFacingAngle m_default =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(
                            SwerveModule.DriveRequestType.OpenLoopVoltage); // Or OpenLoopDutyCycle


    public static Pigeon2 pigeon2 = new Pigeon2(CANMappings.PIGEON_CAN_ID);
  Translation2d m_frontLeftLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(10.875));
  Translation2d m_frontRightLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(-10.875));
  Translation2d m_backLeftLocation =
      new Translation2d(Units.inchesToMeters(-10.875), Units.inchesToMeters(10.875));
  Translation2d m_backRightLocation =
      new Translation2d(Units.inchesToMeters(-10.855), Units.inchesToMeters(-10.875));
  public SwerveDrivePoseEstimator m_odometry =
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

    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putData("Field", m_field);

    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          m_field.getObject("path").setPoses(poses);
        });

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public void updateTelemetry() {
    m_field.setRobotPose(drivetrain.getState().Pose);
  }

  @Logged Pose2d estimatedPosition = m_odometry.getEstimatedPosition();
    @Logged private double goalAngle = drivetrain.getState().ModuleTargets[1].angle.getRotations();
@Logged private double actualAngle = drivetrain.getState().ModuleStates[1].angle.getRotations();
  private void configureBindings() {
    final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(Units.inchesToMeters(59.0), 0.18);
    map.put(Units.inchesToMeters(76.5), 0.155);
    map.put(Units.inchesToMeters(96.5), 0.142);
    map.put(Units.inchesToMeters(125.5), 0.13);
    map.put(Units.inchesToMeters(169.5), 0.12);
    map.put(Units.inchesToMeters(210.5), 0.118);

    NamedCommands.registerCommand(
        "idle",
        Commands.run(() -> intake.stopIntake(), intake)
            .alongWith(
                Commands.run(() -> shooter.stopShooter(), shooter)
                    .alongWith(Commands.run(() -> pivot.pivotDefault(), pivot))));
    NamedCommands.registerCommand(
        "high goal shoot",
        Commands.run(() -> intake.intake(), intake)
            .alongWith(
                Commands.run(() -> shooter.shoot(), shooter)
                    .alongWith(
                        Commands.run(
                            () ->
                                pivot.setPivotAngleRot(
                                    m_odometry
                                        .getEstimatedPosition()
                                        .getTranslation()
                                        .getDistance(pivot.getCosmicConverterTranslation(false))),
                            pivot))));
    NamedCommands.registerCommand(
        "intake",
        Commands.run(() -> intake.intake(), intake)
            .alongWith(
                Commands.run(() -> shooter.stopShooter(), shooter)
                    .alongWith(Commands.run(() -> pivot.setPivotAngleRot(0.0), pivot))));

    // Default commands
    pivot.setDefaultCommand(Commands.run(() -> pivot.pivotDefault(), pivot));
    shooter.setDefaultCommand(Commands.run(() -> shooter.stopShooter(), shooter));
    intake.setDefaultCommand(Commands.run(() -> intake.stopIntake(), intake));
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -controller.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X
                    .withRotationalRate(
                        -controller.getRightX()
                            * MaxAngularRate))); // Drive counterclockwise with negative X

    // Testing - Commented out until need to be used
    // Pivot to angle based off distance
    //    controller
    //        .povUp()
    //        .whileTrue(
    //            (Commands.run(
    //                () ->
    //                    pivot.setPivotAngleRot(
    //                        map.get(
    //                            drivetrain
    //                                .getState()
    //                                .Pose
    //                                .getTranslation()
    //                                .getDistance(pivot.getCosmicConverterTranslation(false)))))));
    //      controller
    //              .povUp()
    //              .whileTrue(
    //                      (Commands.run(
    //                              () ->
    //                                      pivot.setPivotAngleRot(
    //                                              map.get(Units.inchesToMeters(132))))));

    // Zero pivot
    controller.povLeft().onTrue(Commands.runOnce(() -> pivot.zeroPivot()));
    //
    //
    // Shoot
    //    controller
    //        .rightTrigger()
    //        .whileTrue(
    //            Commands.run((() -> intake.runKicker(-0.7)))
    //                .withTimeout(0.5)
    //                .andThen(
    //                    Commands.run(() -> shooter.shoot())
    //                        .alongWith(Commands.run(() -> intake.intake()))));
    //    // Intake
    //    controller.leftTrigger().whileTrue(Commands.run(() -> intake.intake()));

    // Specialized commands
    // auto align with inner cosmic converter and raise pivot
    controller.leftBumper().toggleOnTrue(pivot.getCosmicConverter(controller.rightTrigger(), true));
    // auto align with outer cosmic converter
    controller
        .leftTrigger().toggleOnTrue(pivot.getCosmicConverter(controller.rightTrigger(), false));
    controller.rightTrigger().toggleOnFalse(pivot.defaults());
    // Intake
    controller
        .a()
        .toggleOnTrue(Commands.run(() -> intake.intake()).alongWith(pivot.lowScore(0.31)));

    // Outtake
    controller.b().toggleOnTrue(Commands.run(() -> intake.outtake()));
    // Low score
    controller.x().toggleOnTrue(pivot.lowScore(0.0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void zeroPigeon() {
    Pigeon2 pigeon = new Pigeon2(CANMappings.PIGEON_CAN_ID);
    pigeon.reset();
  }
}
