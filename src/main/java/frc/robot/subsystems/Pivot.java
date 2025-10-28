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

    leftPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 3000;
    rightPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 3000;
    leftPivotConfig.MotionMagic.MotionMagicAcceleration = 500;
    rightPivotConfig.MotionMagic.MotionMagicAcceleration = 500;

    leftPivotConfig.Slot0.kP = 0.0;
    leftPivotConfig.Slot0.kI = 0.0;
    leftPivotConfig.Slot0.kD = 0.0;
    leftPivotConfig.Slot0.kS = 0.0;
    leftPivotConfig.Slot0.kG = 0.0;
    leftPivotConfig.Slot0.kV = 0.0;
    leftPivotConfig.Slot0.kA = 0.0;

    rightPivotConfig.Slot0.kP = 0.0;
    rightPivotConfig.Slot0.kI = 0.0;
    rightPivotConfig.Slot0.kD = 0.0;
    rightPivotConfig.Slot0.kS = 0.0;
    rightPivotConfig.Slot0.kG = 0.0;
    rightPivotConfig.Slot0.kV = 0.0;
    rightPivotConfig.Slot0.kA = 0.0;

    leftPivotConfig.Feedback.SensorToMechanismRatio = 25; // gear ratio
    rightPivotConfig.Feedback.SensorToMechanismRatio = 25;

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
