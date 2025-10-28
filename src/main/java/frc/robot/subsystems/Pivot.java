package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.CANMappings;
import frc.robot.config.PivotConfig;

public class Pivot {
  protected TalonFX mPivotLeft;
  protected TalonFX mPivotRight;
  protected Follower follower;

  private double setpoint;
  private double currentAngleDegrees;

  public Pivot() {
    mPivotLeft = new TalonFX(CANMappings.K_PIVOT_LEFT_ID);
    mPivotRight = new TalonFX(CANMappings.K_PIVOT_RIGHT_ID);

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

    leftPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mPivotLeft.getConfigurator().apply(leftPivotConfig);
    mPivotRight.getConfigurator().apply(rightPivotConfig);

    follower = new Follower(CANMappings.K_PIVOT_LEFT_ID, true);
  }

  // angleSetpoint in rotations
  public void setPivotAngle(Rotation2d angleSetpoint) {
    setpoint = angleSetpoint.getRotations();

    mPivotLeft.setControl(new MotionMagicVoltage(angleSetpoint.getRotations()));
    mPivotRight.setControl(follower);
  }

  // placeholders, incomplete
  public Rotation2d getHighShootAngle() {
    // add calculations
    return Rotation2d.fromDegrees(0);
  }

  public Rotation2d getLowShootAngle() {
    // add calculations
    return Rotation2d.fromDegrees(0);
  }

  public boolean pivotAtSetpoint() {
    return Math.abs((getPivotAngleRotations() - setpoint)) <= PivotConfig.K_PIVOT_ANGLE_TOLERANCE;
  }

  public double getPivotAngleDegrees() {
    currentAngleDegrees = mPivotLeft.getPosition().getValueAsDouble() * 360;
    return currentAngleDegrees;
  }

  public double getPivotAngleRotations() {
    currentAngleDegrees = mPivotLeft.getPosition().getValueAsDouble();
    return currentAngleDegrees;
  }

  public void zeroPivot() {
    mPivotLeft.setPosition(0.0);
    mPivotRight.setPosition(0.0);
  }

  public void stopPivot() {
    mPivotLeft.stopMotor();
    mPivotRight.stopMotor();
  }

  // @Override
  public void periodic() {
    SmartDashboard.putBoolean("Pivot At Setpoint", pivotAtSetpoint());
    SmartDashboard.putNumber("Pivot Actual Angle", getPivotAngleDegrees());
    SmartDashboard.putNumber("Pivot Goal Angle", setpoint);
    SmartDashboard.putNumber("Pivot Error", mPivotLeft.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Left Pivot Motor Output %", mPivotLeft.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putNumber(
        "Right Pivot Motor Output %", mPivotRight.getMotorOutputStatus().getValueAsDouble());
  }
}
