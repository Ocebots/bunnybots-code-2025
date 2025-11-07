package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.CANMappings;
import frc.robot.config.ShooterConfig;

public class Shooter {
  protected TalonFX mTopShooterMotor;
  protected TalonFX mBottomShooterMotor;

  public Shooter() {
    mTopShooterMotor = new TalonFX(CANMappings.K_SHOOTER_TOP_ID);
    mBottomShooterMotor = new TalonFX(CANMappings.K_SHOOTER_BOTTOM_ID);

    TalonFXConfiguration topConfig = new TalonFXConfiguration();
    TalonFXConfiguration bottomConfig = new TalonFXConfiguration();

    topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConfig.K_SHOOTER_TOP_MOTORS_SUPPLY_CURRENT_LIMIT;
    bottomConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConfig.K_SHOOTER_BOTTOM_MOTORS_SUPPLY_CURRENT_LIMIT;
    topConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConfig.K_SHOOTER_TOP_MOTORS_STATOR_CURRENT_LIMIT;
    bottomConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConfig.K_SHOOTER_BOTTOM_MOTORS_STATOR_CURRENT_LIMIT;
    topConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConfig.K_SHOOTER_TOP_AND_SHOOTER_BOTTOM_MAX_CRUISE_VELOCITY;
    topConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConfig.K_SHOOTER_TOP_AND_SHOOTER_BOTTOM_TARGET_ACCELERATION;
    bottomConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConfig.K_SHOOTER_TOP_AND_SHOOTER_BOTTOM_MAX_CRUISE_VELOCITY;
    bottomConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConfig.K_SHOOTER_TOP_AND_SHOOTER_BOTTOM_TARGET_ACCELERATION;

    topConfig.Slot0.kP = ShooterConfig.K_SHOOTER_TOP_P;
    topConfig.Slot0.kI = ShooterConfig.K_SHOOTER_TOP_I;
    topConfig.Slot0.kD = ShooterConfig.K_SHOOTER_TOP_D;
    topConfig.Slot0.kS = ShooterConfig.K_SHOOTER_TOP_S;
    topConfig.Slot0.kV = ShooterConfig.K_SHOOTER_TOP_V;
    topConfig.Slot0.kA = ShooterConfig.K_SHOOTER_TOP_A;
    bottomConfig.Slot0.kP = ShooterConfig.K_SHOOTER_BOTTOM_P;
    bottomConfig.Slot0.kI = ShooterConfig.K_SHOOTER_BOTTOM_I;
    bottomConfig.Slot0.kD = ShooterConfig.K_SHOOTER_BOTTOM_D;
    bottomConfig.Slot0.kS = ShooterConfig.K_SHOOTER_BOTTOM_S;
    bottomConfig.Slot0.kV = ShooterConfig.K_SHOOTER_BOTTOM_V;
    bottomConfig.Slot0.kA = ShooterConfig.K_SHOOTER_BOTTOM_A;

    topConfig.Feedback.SensorToMechanismRatio = ShooterConfig.K_SHOOTER_TOP_GEAR_RATIO;
    bottomConfig.Feedback.SensorToMechanismRatio = ShooterConfig.K_SHOOTER_BOTTOM_GEAR_RATIO;

    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mTopShooterMotor.getConfigurator().apply(topConfig);
    mBottomShooterMotor.getConfigurator().apply(bottomConfig);
  }

  public void shoot(double velocity) {
    mTopShooterMotor.setControl(new VelocityVoltage(velocity));
    mBottomShooterMotor.setControl(new VelocityVoltage(velocity));
  }

  public void stop() {
    mTopShooterMotor.stopMotor();
    mBottomShooterMotor.stopMotor();
  }
}
