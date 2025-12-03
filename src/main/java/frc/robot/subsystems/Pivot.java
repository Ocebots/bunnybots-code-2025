package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.config.CANMappings;
import frc.robot.config.PivotConfig;
import frc.robot.config.TunerConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

@Logged
public class Pivot extends SubsystemBase {
  protected TalonFX mPivotLeft;
  protected TalonFX mPivotRight;
  protected Follower follower;
  protected CommandSwerveDrivetrain drivetrain;
  InterpolatingDoubleTreeMap map1 = new InterpolatingDoubleTreeMap();

  private double currentAngle;

  public Pivot() {
    mPivotLeft = new TalonFX(CANMappings.K_PIVOT_LEFT_ID);
    mPivotRight = new TalonFX(CANMappings.K_PIVOT_RIGHT_ID);

    drivetrain = TunerConstants.createDrivetrain();

    TalonFXConfiguration leftPivotConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightPivotConfig = new TalonFXConfiguration();

    leftPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftPivotConfig.CurrentLimits.StatorCurrentLimit =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_STATOR_CURRENT_LIMIT;
    leftPivotConfig.CurrentLimits.SupplyCurrentLimit =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_SUPPLY_CURRENT_LIMIT;
    ;

    rightPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightPivotConfig.CurrentLimits.StatorCurrentLimit =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_STATOR_CURRENT_LIMIT;
    ;
    rightPivotConfig.CurrentLimits.SupplyCurrentLimit =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_SUPPLY_CURRENT_LIMIT;
    ;

    leftPivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_MAX_CRUISE_VELOCITY;
    rightPivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_MAX_CRUISE_VELOCITY;
    leftPivotConfig.MotionMagic.MotionMagicAcceleration =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_TARGET_ACCELERATION;
    rightPivotConfig.MotionMagic.MotionMagicAcceleration =
        PivotConfig.K_LEFT_AND_RIGHT_PIVOT_TARGET_ACCELERATION;

    leftPivotConfig.Slot0.kP = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_P;
    leftPivotConfig.Slot0.kI = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_I;
    leftPivotConfig.Slot0.kD = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_D;
    leftPivotConfig.Slot0.kS = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_S;
    leftPivotConfig.Slot0.kG = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_G;
    leftPivotConfig.Slot0.kV = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_V;
    leftPivotConfig.Slot0.kA = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_A;

    rightPivotConfig.Slot0.kP = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_P;
    rightPivotConfig.Slot0.kI = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_I;
    rightPivotConfig.Slot0.kD = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_D;
    rightPivotConfig.Slot0.kS = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_S;
    rightPivotConfig.Slot0.kG = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_G;
    rightPivotConfig.Slot0.kV = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_V;
    rightPivotConfig.Slot0.kA = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_A;

    leftPivotConfig.Feedback.SensorToMechanismRatio =
        PivotConfig.K_LEFT_PIVOT_GEAR_RATIO; // gear ratio
    rightPivotConfig.Feedback.SensorToMechanismRatio = PivotConfig.K_RIGHT_PIVOT_GEAR_RATIO;

    leftPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // leftPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    mPivotLeft.getConfigurator().apply(leftPivotConfig);
    mPivotRight.getConfigurator().apply(rightPivotConfig);

    // follower = new Follower(CANMappings.K_PIVOT_LEFT_ID, false);
  }

  public void setPivotAngle(Rotation2d angleSetpoint) {
    mPivotLeft.setControl(new MotionMagicVoltage(angleSetpoint.getRotations()));
    mPivotRight.setControl(follower);
  }

  public void setPivotAngleRot(double rotation) {
    mPivotLeft.setControl(new MotionMagicVoltage(-rotation));
    mPivotRight.setControl(new MotionMagicVoltage(rotation));
    // mPivotRight.setControl(follower);
  }

  public void zeroPivot() {
    mPivotLeft.setPosition(0.0);
    mPivotRight.setPosition(0.0);
  }

  public void stopPivot() {
    mPivotLeft.stopMotor();
    mPivotRight.stopMotor();
  }

  public boolean pivotAtSetpoint() {
    return Math.abs(mPivotLeft.getClosedLoopError().getValueAsDouble())
        <= PivotConfig.K_PIVOT_ANGLE_TOLERANCE;
  }

  public Rotation2d getHighAngle(Translation2d location) {
    // location: the cosmic converter we're shooting on - 1 is blue inner, 2 is blue outer, 3 is red
    // inner, 4 is red outer
    // want 5-8 calibrations (distance, angle)
    // in,
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(59.0, 0.18);
    map.put(76.5, 0.155);
    map.put(96.5, 0.142);
    map.put(125.5, 0.13);
    map.put(169.5, 0.12);
    map.put(210.5, 0.118);

    double distance =
        Math.sqrt(
            Math.pow(location.getX() - drivetrain.getState().Pose.getX(), 2)
                + Math.pow(location.getY() - drivetrain.getState().Pose.getY(), 2));
    return Rotation2d.fromDegrees(map.get(distance));
  }

  public double getPivotAngleDegrees() {
    currentAngle = mPivotLeft.getPosition().getValueAsDouble();
    currentAngle = currentAngle * 360;

    return currentAngle;
  }

  private final SwerveRequest.FieldCentricFacingAngle m_faceAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // Or OpenLoopDutyCycle

  public static int getAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return 0;
      }
      if (alliance.get() == DriverStation.Alliance.Red) {
        return 1;
      }
    }
    System.out.println("no alliance detected: likely causing many errors");
    return -1;
  }

  public static Translation2d getLocation(int innerouter) {
    // ArrayList values: 0 - blue inner, 1 - blue outer, 2 - red inner, 3 - red outer
    // innerouter: 0 - outer, 1 - inner
    // getAlliance(): blue - 0, red - 1

    List<Translation2d> locations =
        new ArrayList<>(
            Arrays.asList(
                new Translation2d(4.0, 196.125),
                new Translation2d(4.0, 20.5),
                new Translation2d(644.0, 196.125),
                new Translation2d(644.0, 20.5))); // same order as explained above

    if (innerouter == 1 & Pivot.getAlliance() == 0) { // blue inner
      return locations.get(0);
    }
    if (innerouter == 0 & Pivot.getAlliance() == 0) { // blue outer
      return locations.get(1);
    }
    if (innerouter == 1 & Pivot.getAlliance() == 1) { // red inner
      return locations.get(2);
    }
    if (innerouter == 0 & Pivot.getAlliance() == 1) { // red outer
      return locations.get(3);
    }
    System.out.println("error in getLocation in pivot subsystem");
    return new Translation2d(0.0, 0.0);
  }

  public DrivetrainCommand.Position getClosestCosmicConverterDrivetrain() {
    Translation2d currentLocation =
        new Translation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
    // ArrayList values: 0 - blue inner, 1 - blue outer, 2 - red inner, 3 - red outer
    // innerouter: 0 - outer, 1 - inner
    // getAlliance(): blue - 0, red - 1

    List<Translation2d> locations =
        new ArrayList<>(
            Arrays.asList(
                new Translation2d(4.0, 196.125),
                new Translation2d(4.0, 20.5),
                new Translation2d(644.0, 196.125),
                new Translation2d(644.0, 20.5))); // same order as explained above

    if (Pivot.getAlliance() == 1) { // red
      if (Math.sqrt(
              Math.pow(locations.get(2).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(2).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(3).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(3).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER; // outer
      } else {
        return DrivetrainCommand.Position.INNER_COSMIC_CONVERTER; // inner
      }
    } else if (Pivot.getAlliance() == 0) { // blue
      if (Math.sqrt(
              Math.pow(locations.get(0).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(0).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(1).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(1).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER; // outer
      } else {
        return DrivetrainCommand.Position.INNER_COSMIC_CONVERTER; // inner
      }
    } else {
      System.out.println("error in getClosestCosmicConverter() in Pivot");
      return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER;
    }
  }

  public DrivetrainCommand.Position getFarthestCosmicConverterDrivetrain() {
    Translation2d currentLocation =
        new Translation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
    // ArrayList values: 0 - blue inner, 1 - blue outer, 2 - red inner, 3 - red outer
    // innerouter: 0 - outer, 1 - inner
    // getAlliance(): blue - 0, red - 1

    List<Translation2d> locations =
        new ArrayList<>(
            Arrays.asList(
                new Translation2d(4.0, 196.125),
                new Translation2d(4.0, 20.5),
                new Translation2d(644.0, 196.125),
                new Translation2d(644.0, 20.5))); // same order as explained above

    if (Pivot.getAlliance() == 1) { // red
      if (Math.sqrt(
              Math.pow(locations.get(2).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(2).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(3).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(3).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return DrivetrainCommand.Position.INNER_COSMIC_CONVERTER; // inner
      } else {
        return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER; // outer
      }
    } else if (Pivot.getAlliance() == 0) { // blue
      if (Math.sqrt(
              Math.pow(locations.get(0).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(0).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(1).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(1).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return DrivetrainCommand.Position.INNER_COSMIC_CONVERTER; // inner
      } else {
        return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER; // outer
      }
    } else {
      System.out.println("error in getClosestCosmicConverter() in Pivot");
      return DrivetrainCommand.Position.OUTER_COSMIC_CONVERTER;
    }
  }

  public PivotCommand.Position getClosestCosmicConverterPivot() {
    Translation2d currentLocation =
        new Translation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
    // ArrayList values: 0 - blue inner, 1 - blue outer, 2 - red inner, 3 - red outer
    // innerouter: 0 - outer, 1 - inner
    // getAlliance(): blue - 0, red - 1

    List<Translation2d> locations =
        new ArrayList<>(
            Arrays.asList(
                new Translation2d(4.0, 196.125),
                new Translation2d(4.0, 20.5),
                new Translation2d(644.0, 196.125),
                new Translation2d(644.0, 20.5))); // same order as explained above

    if (Pivot.getAlliance() == 1) { // red
      if (Math.sqrt(
              Math.pow(locations.get(2).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(2).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(3).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(3).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return PivotCommand.Position.OUTER_HIGH_SHOOT; // outer
      } else {
        return PivotCommand.Position.INNER_HIGH_SHOOT; // inner
      }
    } else if (Pivot.getAlliance() == 0) { // blue
      if (Math.sqrt(
              Math.pow(locations.get(0).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(0).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(1).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(1).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return PivotCommand.Position.OUTER_HIGH_SHOOT; // outer
      } else {
        return PivotCommand.Position.INNER_HIGH_SHOOT; // inner
      }
    } else {
      System.out.println("error in getClosestCosmicConverter() in Pivot");
      return PivotCommand.Position.OUTER_HIGH_SHOOT;
    }
  }

  public PivotCommand.Position getFarthestCosmicConverterPivot() {
    Translation2d currentLocation =
        new Translation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
    // ArrayList values: 0 - blue inner, 1 - blue outer, 2 - red inner, 3 - red outer
    // innerouter: 0 - outer, 1 - inner
    // getAlliance(): blue - 0, red - 1

    List<Translation2d> locations =
        new ArrayList<>(
            Arrays.asList(
                new Translation2d(4.0, 196.125),
                new Translation2d(4.0, 20.5),
                new Translation2d(644.0, 196.125),
                new Translation2d(644.0, 20.5))); // same order as explained above

    if (Pivot.getAlliance() == 1) { // red
      if (Math.sqrt(
              Math.pow(locations.get(2).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(2).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(3).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(3).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return PivotCommand.Position.INNER_HIGH_SHOOT; // inner
      } else {
        return PivotCommand.Position.OUTER_HIGH_SHOOT; // outer
      }
    } else if (Pivot.getAlliance() == 0) { // blue
      if (Math.sqrt(
              Math.pow(locations.get(0).getX() - drivetrain.getState().Pose.getX(), 2)
                  + Math.pow(locations.get(0).getY() - drivetrain.getState().Pose.getY(), 2))
          > Math.pow(locations.get(1).getX() - drivetrain.getState().Pose.getX(), 2)
              + Math.pow(locations.get(1).getY() - drivetrain.getState().Pose.getY(), 2)) {
        return PivotCommand.Position.INNER_HIGH_SHOOT; // inner
      } else {
        return PivotCommand.Position.OUTER_HIGH_SHOOT; // outer
      }
    } else {
      System.out.println("error in getClosestCosmicConverter() in Pivot");
      return PivotCommand.Position.OUTER_HIGH_SHOOT;
    }
  }

  public Command getCosmicConverter(boolean isInner) {
    Optional<DriverStation.Alliance> alliance1 = DriverStation.getAlliance();
    Translation2d cosmicConverter = new Translation2d();
    if (alliance1.isPresent()) {
      if (alliance1.get() == DriverStation.Alliance.Blue) {
        if (isInner) {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(196.125));
        } else {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(20.5));
        }
      }
      if (alliance1.get() == DriverStation.Alliance.Red) {
        if (isInner) {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(196.125));
        } else {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(20.5));
        }
      }

      Rotation2d heading = drivetrain.getState().Pose.getRotation();

      // shooter offset in robot frame (meters)
      double shooterOffsetX = 0.20; // forward
      double shooterOffsetY = -0.10; // right

      // convert to field frame
      double shooterX =
          drivetrain.getState().Pose.getX()
              + shooterOffsetX * heading.getCos()
              - shooterOffsetY * heading.getSin();

      double shooterY =
          drivetrain.getState().Pose.getY()
              + shooterOffsetX * heading.getSin()
              + shooterOffsetY * heading.getCos();

      // compute target angle
      Rotation2d aimAngle =
          new Rotation2d(
              Math.atan2(cosmicConverter.getY() - shooterY, cosmicConverter.getX() - shooterX));

      return Commands.runOnce(
          () ->
              drivetrain.setControl(
                  m_faceAngle.withVelocityX(0.1).withVelocityY(0.1).withTargetDirection(aimAngle)));
    } else {
      cosmicConverter = null;
      System.out.println("no alliance detected: likely causing many errors");
      return null;
    }
  }

  public Translation2d getCosmicConverterTranslation(boolean isInner) {
    Optional<DriverStation.Alliance> alliance1 = DriverStation.getAlliance();
    Translation2d cosmicConverter = new Translation2d();
    if (alliance1.isPresent()) {
      if (alliance1.get() == DriverStation.Alliance.Blue) {
        if (isInner) {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(196.125));
        } else {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(20.5));
        }
      }
      if (alliance1.get() == DriverStation.Alliance.Red) {
        if (isInner) {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(196.125));
        } else {
          cosmicConverter =
              new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(20.5));
        }
      }
    }
    return cosmicConverter;
  }
}
