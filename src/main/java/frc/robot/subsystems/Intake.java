package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

public class Intake extends SubsystemBase {
  protected TalonFX mInitialIntake;
  protected TalonFX mKickerIntake;

  public Intake() {
    mInitialIntake = new TalonFX(CANMappings.M_INITIAL_INTAKE_ID);
    mKickerIntake = new TalonFX(CANMappings.M_KICKER_INTAKE_ID);

    TalonFXConfiguration initialIntakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerIntakeConfig = new TalonFXConfiguration();

    initialIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    initialIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    initialIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.M_INITIAL_INTAKE_SUPPLY_CURRENT_LIMIT;
    initialIntakeConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConfig.M_INITIAL_INTAKE_STATOR_CURRENT_LIMIT;

    kickerIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.M_KICKER_INTAKE_SUPPLY_CURRENT_LIMIT;
    kickerIntakeConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConfig.M_KICKER_INTAKE_STATOR_CURRENT_LIMIT;

    initialIntakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConfig.M_INITIAL_AND_KICKER_MAX_CRUISE_VELOCITY;
    kickerIntakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConfig.M_INITIAL_AND_KICKER_MAX_CRUISE_VELOCITY;
    initialIntakeConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConfig.M_INITIAL_AND_KICKER_TARGET_ACCELERATION;
    kickerIntakeConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConfig.M_INITIAL_AND_KICKER_TARGET_ACCELERATION;

    initialIntakeConfig.Slot0.kP = IntakeConfig.M_INITIAL_INTAKE_P;
    initialIntakeConfig.Slot0.kI = IntakeConfig.M_INITIAL_INTAKE_I;
    initialIntakeConfig.Slot0.kD = IntakeConfig.M_INITIAL_INTAKE_D;
    initialIntakeConfig.Slot0.kS = IntakeConfig.M_INITIAL_INTAKE_S;
    initialIntakeConfig.Slot0.kV = IntakeConfig.M_INITIAL_INTAKE_V;
    initialIntakeConfig.Slot0.kA = IntakeConfig.M_INITIAL_INTAKE_A;

    kickerIntakeConfig.Slot0.kP = IntakeConfig.M_KICKER_INTAKE_P;
    kickerIntakeConfig.Slot0.kI = IntakeConfig.M_KICKER_INTAKE_I;
    kickerIntakeConfig.Slot0.kD = IntakeConfig.M_KICKER_INTAKE_D;
    kickerIntakeConfig.Slot0.kS = IntakeConfig.M_KICKER_INTAKE_S;
    kickerIntakeConfig.Slot0.kV = IntakeConfig.M_KICKER_INTAKE_V;
    kickerIntakeConfig.Slot0.kA = IntakeConfig.M_KICKER_INTAKE_A;

    initialIntakeConfig.Feedback.SensorToMechanismRatio = IntakeConfig.M_INITIAL_INTAKE_GEAR_RATIO;
    kickerIntakeConfig.Feedback.SensorToMechanismRatio = IntakeConfig.M_KICKER_INTAKE_GEAR_RATIO;

    initialIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mInitialIntake.getConfigurator().apply(initialIntakeConfig);
    mKickerIntake.getConfigurator().apply(kickerIntakeConfig);
  }

  public void intake(double velocity) {
    mInitialIntake.setControl(new VelocityVoltage(velocity));
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  public void runKicker(double velocity) {
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  public void runInitial(double velocity) {
    mInitialIntake.setControl(new VelocityVoltage(velocity));
  }

  public void stopIntake() {
    mInitialIntake.stopMotor();
    mKickerIntake.stopMotor();
  }

  public void stopInitial() {
    mInitialIntake.stopMotor();
  }

  public void stopKicker() {
    mKickerIntake.stopMotor();
  }
}
