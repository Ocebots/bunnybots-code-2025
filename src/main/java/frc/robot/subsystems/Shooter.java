package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.ShooterConfig;

public class Shooter extends SubsystemBase {
  protected TalonFX mTopShooter;
  protected TalonFX mBottomShooter;

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
        ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_STATOR_CURRENT_LIMIT;
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
  }

  // Velocity is rotations per second of motor accounting for SensorToMechanismRatio
  public void shoot(double velocity) {
    mTopShooter.setControl(new VelocityVoltage(velocity));
    mBottomShooter.setControl(new VelocityVoltage(velocity));
  }

  public void stopShooter() {
    mTopShooter.stopMotor();
    mBottomShooter.stopMotor();
  }
}
