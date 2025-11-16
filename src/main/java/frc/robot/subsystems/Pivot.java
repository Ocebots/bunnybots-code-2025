package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    leftPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ;

    mPivotLeft.getConfigurator().apply(leftPivotConfig);
    mPivotRight.getConfigurator().apply(rightPivotConfig);

    follower = new Follower(CANMappings.K_PIVOT_LEFT_ID, true);
  }

  public void setPivotAngle(Rotation2d angleSetpoint) {
    mPivotLeft.setControl(new MotionMagicVoltage(angleSetpoint.getRotations()));
    mPivotRight.setControl(follower);
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

  public Rotation2d getHighAngle(int location) {
    // location: the cosmic converter we're shooting on - 1 is blue inner, 2 is blue outer, 3 is red
    // inner, 4 is red outer
    // want 5-8 calibrations (distance, angle)
    double cosmicConverterX = 0.0;
    double cosmicConverterY = 0.0;
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(0.0, 0.0);

    List<Double> locations =
        new ArrayList<>(
            Arrays.asList(
                Units.inchesToMeters(4.0),
                Units.inchesToMeters(196.125),
                Units.inchesToMeters(4.0),
                Units.inchesToMeters(20.5),
                Units.inchesToMeters(644.0),
                Units.inchesToMeters(196.125),
                Units.inchesToMeters(644.0),
                Units.inchesToMeters(20.5))); // same order as explained above
    if (location == 1) {
      cosmicConverterX = locations.get(0);
      cosmicConverterY = locations.get(1);
    }
    if (location == 2) {
      cosmicConverterX = locations.get(2);
      cosmicConverterY = locations.get(3);
    }
    if (location == 3) {
      cosmicConverterX = locations.get(4);
      cosmicConverterY = locations.get(5);
    }
    if (location == 4) {
      cosmicConverterX = locations.get(6);
      cosmicConverterY = locations.get(7);
    }

    double distance =
        Math.sqrt(
            Math.pow(cosmicConverterX - drivetrain.getState().Pose.getX(), 2)
                + Math.pow(cosmicConverterY - drivetrain.getState().Pose.getY(), 2));
    return Rotation2d.fromDegrees(map.get(distance));
  }

  public double getPivotAngleDegrees() {
    currentAngle = mPivotLeft.getPosition().getValueAsDouble();
    currentAngle = currentAngle * 360;

    return currentAngle;
  }

  public int getAlliance() {
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
}
