package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.CANMappings;
import frc.robot.config.ShooterConfig;

public class Shooter {
  protected TalonFX mTopShooter;
  protected TalonFX mBottomShooter;
  protected Follower follower;

  private double tolerance;
  private double setpoint;

  public Shooter() {
    mTopShooter = new TalonFX(CANMappings.K_TOP_SHOOTER_ID);
    mBottomShooter = new TalonFX(CANMappings.K_BOTTOM_SHOOTER_ID);

    TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
    TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();

    topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    topShooterConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_STATOR_CURRENT_LIMIT;
    topShooterConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_SUPPLY_CURRENT_LIMIT;

    bottomShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_SUPPLY_CURRENT_LIMIT;
    bottomShooterConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_SUPPLY_CURRENT_LIMIT;

    topShooterConfig.Slot0.kP = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_P;
    topShooterConfig.Slot0.kI = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_I;
    topShooterConfig.Slot0.kD = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_D;
    topShooterConfig.Slot0.kS = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_S;
    topShooterConfig.Slot0.kV = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_V;
    topShooterConfig.Slot0.kA = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_A;

    bottomShooterConfig.Slot0.kP = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_P;
    bottomShooterConfig.Slot0.kI = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_I;
    bottomShooterConfig.Slot0.kD = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_D;
    bottomShooterConfig.Slot0.kS = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_S;
    bottomShooterConfig.Slot0.kV = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_V;
    bottomShooterConfig.Slot0.kA = ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_A;

    topShooterConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_MAX_CRUISE_VELOCITY;
    topShooterConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_MAX_CRUISE_VELOCITY;
    bottomShooterConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_TARGET_ACCELERATION;
    bottomShooterConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_TARGET_ACCELERATION;

    topShooterConfig.Feedback.SensorToMechanismRatio =
        ShooterConfig.K_TOP_SHOOTER_GEAR_RATIO; // gear ratio (wheel rps)
    bottomShooterConfig.Feedback.SensorToMechanismRatio = ShooterConfig.K_BOTTOM_SHOOTER_GEAR_RATIO;

    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mTopShooter.getConfigurator().apply(topShooterConfig);
    mBottomShooter.getConfigurator().apply(bottomShooterConfig);

    follower = new Follower(CANMappings.K_TOP_SHOOTER_ID, false);
  }

  // Velocity is rotations per second of motor accounting for SensorToMechanismRatio
  public void shoot(double velocity) {
    setpoint = velocity;
    tolerance = Math.max(1.0, velocity * 0.02);
    mTopShooter.setControl(new VelocityVoltage(velocity));
    mBottomShooter.setControl(follower);
  }

  public void stopShooter() {
    tolerance = 0;
    setpoint = 0;
    mTopShooter.stopMotor();
    mBottomShooter.stopMotor();
  }

  public boolean shooterActive() {
    return Math.abs(mTopShooter.getVelocity().getValueAsDouble()) > 0
        || Math.abs(mBottomShooter.getVelocity().getValueAsDouble()) > 0;
  }

  public double actualTopShooterMotorSpeedRPS() {
    return mTopShooter.getVelocity().getValueAsDouble();
  }

  public double actualBottomShooterMotorSpeedRPS() {
    return mBottomShooter.getVelocity().getValueAsDouble();
  }

  public double goalShootSpeedRPS() {
    return mTopShooter.getClosedLoopReference().getValueAsDouble();
  }

  public boolean topShooterMotorAtGoalSpeed() {
    return Math.abs(actualTopShooterMotorSpeedRPS() - setpoint) <= tolerance;
  }

  public boolean bottomShooterMotorAtGoalSpeed() {
    return Math.abs(actualBottomShooterMotorSpeedRPS() - setpoint) <= tolerance;
  }

  public boolean shooterReady() {
    return topShooterMotorAtGoalSpeed() && bottomShooterMotorAtGoalSpeed();
  }

  // @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Active", shooterActive());
    SmartDashboard.putNumber(
        "Top Shooter Motor Actual Speed (Rotations/Second)", actualTopShooterMotorSpeedRPS());
    SmartDashboard.putNumber(
        "Bottom Shooter Motor Actual Speed (Rotations/Second)", actualBottomShooterMotorSpeedRPS());
    SmartDashboard.putNumber("Shooter Goal Speed (Rotations/Second)", goalShootSpeedRPS());
    SmartDashboard.putBoolean("Top Shooter Motor Is At Goal Speed", topShooterMotorAtGoalSpeed());
    SmartDashboard.putBoolean(
        "Bottom Shooter Motor Is At Goal Speed", bottomShooterMotorAtGoalSpeed());
    SmartDashboard.putNumber(
        "Top Shooter Motor Speed Error", mTopShooter.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Bottom Shooter Motor Speed Error", mBottomShooter.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Top Shooter Motor Output %", mTopShooter.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putNumber(
        "Bottom Shooter Motor Output %", mBottomShooter.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter Ready", shooterReady());
  }
}
