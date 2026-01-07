package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

@Logged
public class Intake extends SubsystemBase {
  protected TalonFX mInitialIntake;
  protected TalonFX mKickerIntake;

  public Intake() {
    mInitialIntake = new TalonFX(CANMappings.K_INITIAL_INTAKE_ID);
    mKickerIntake = new TalonFX(CANMappings.K_KICKER_INTAKE_ID);

    TalonFXConfiguration initialIntakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerIntakeConfig = new TalonFXConfiguration();

    initialIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    initialIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    initialIntakeConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConfig.K_INITIAL_INTAKE_STATOR_CURRENT_LIMIT;
    initialIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.K_INITIAL_INTAKE_SUPPLY_CURRENT_LIMIT;

    kickerIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerIntakeConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConfig.K_KICKER_INTAKE_STATOR_CURRENT_LIMIT;
    kickerIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.K_KICKER_INTAKE_SUPPLY_CURRENT_LIMIT;

    initialIntakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConfig.K_INITIAL_AND_KICKER_INTAKE_MAX_CRUISE_VELOCITY;
    initialIntakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConfig.K_INITIAL_AND_KICKER_INTAKE_MAX_CRUISE_VELOCITY;
    kickerIntakeConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConfig.K_INITIAL_AND_KICKER_INTAKE_TARGET_ACCELERATION;
    kickerIntakeConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConfig.K_INITIAL_AND_KICKER_INTAKE_TARGET_ACCELERATION;

    initialIntakeConfig.Slot0.kP = IntakeConfig.K_INITIAL_INTAKE_P;
    initialIntakeConfig.Slot0.kI = IntakeConfig.K_INITIAL_INTAKE_I;
    initialIntakeConfig.Slot0.kD = IntakeConfig.K_INITIAL_INTAKE_D;
    initialIntakeConfig.Slot0.kS = IntakeConfig.K_INITIAL_INTAKE_S;
    initialIntakeConfig.Slot0.kV = IntakeConfig.K_INITIAL_INTAKE_V;
    initialIntakeConfig.Slot0.kA = IntakeConfig.K_INITIAL_INTAKE_A;

    kickerIntakeConfig.Slot0.kP = IntakeConfig.K_KICKER_INTAKE_P;
    kickerIntakeConfig.Slot0.kI = IntakeConfig.K_KICKER_INTAKE_I;
    kickerIntakeConfig.Slot0.kD = IntakeConfig.K_KICKER_INTAKE_D;
    kickerIntakeConfig.Slot0.kS = IntakeConfig.K_KICKER_INTAKE_S;
    kickerIntakeConfig.Slot0.kV = IntakeConfig.K_KICKER_INTAKE_V;
    kickerIntakeConfig.Slot0.kA = IntakeConfig.K_KICKER_INTAKE_A;

    initialIntakeConfig.Feedback.SensorToMechanismRatio =
        IntakeConfig.K_INITIAL_INTAKE_GEAR_RATIO; // gear ratio
    kickerIntakeConfig.Feedback.SensorToMechanismRatio = IntakeConfig.K_KICKER_INTAKE_GEAR_RATIO;

    initialIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mInitialIntake.getConfigurator().apply(initialIntakeConfig);
    mKickerIntake.getConfigurator().apply(kickerIntakeConfig);
  }

  // Velocity is rotations per second of motor accounting for SensorToMechanismRatio
  public void intake(double initialVelocity, double kickerVelocity) {
    mInitialIntake.setControl(new DutyCycleOut(initialVelocity));
    mKickerIntake.setControl(new DutyCycleOut(kickerVelocity));
  }

  public void intake() {
    mInitialIntake.setControl(new DutyCycleOut(-0.5));
    mKickerIntake.setControl(new DutyCycleOut(-1));
  }

  public void outtake() {
    mInitialIntake.setControl(new DutyCycleOut(0.5));
    mKickerIntake.setControl(new DutyCycleOut(0.7));
  }

  public void runKicker(double velocity) {
    mKickerIntake.setControl(new DutyCycleOut(velocity));
  }

  public void runKicker() {
    mKickerIntake.setControl(new DutyCycleOut(-0.7));
  }

  public void runInitial(double velocity) {
    mInitialIntake.setControl(new DutyCycleOut(velocity));
  }

  public void runInitial() {
    mInitialIntake.setControl(new DutyCycleOut(-0.5));
  }

  public void stopIntake() {
    mInitialIntake.stopMotor();
    mKickerIntake.stopMotor();
  }

  public void stopKicker() {
    mKickerIntake.stopMotor();
  }

  public void stopInitial() {
    mInitialIntake.stopMotor();
  }
}
