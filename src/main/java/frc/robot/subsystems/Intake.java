package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

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
  public void intake(double velocity) {
    mInitialIntake.setControl(new VelocityVoltage(velocity));
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  public void stopIntake() {
    mInitialIntake.stopMotor();
    mKickerIntake.stopMotor();
  }

  public void stopKicker() {
    mKickerIntake.stopMotor();
  }

  public void runKicker(double velocity) {
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  // 0.5 is used to cut out possible noise
  public boolean intakeActive() {
    return Math.abs(mInitialIntake.getVelocity().getValueAsDouble()) > 0.5
        || Math.abs(mKickerIntake.getVelocity().getValueAsDouble()) > 0.5;
  }

  public double actualInitialIntakeMotorSpeedRPS() {
    return mInitialIntake.getVelocity().getValueAsDouble();
  }

  public double actualKickerIntakeMotorSpeedRPS() {
    return mKickerIntake.getVelocity().getValueAsDouble();
  }

  public double goalPIDInitialIntakeSpeedRPS() {
    return mInitialIntake.getClosedLoopReference().getValueAsDouble();
  }

  public double goalPIDKickerIntakeSpeedRPS() {
    return mKickerIntake.getClosedLoopReference().getValueAsDouble();
  }

  // @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Active", intakeActive());
    SmartDashboard.putNumber(
        "Initial Intake Motor Speed (Rotations/Second)", actualInitialIntakeMotorSpeedRPS());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Speed (Rotations/Second)", actualKickerIntakeMotorSpeedRPS());
    SmartDashboard.putNumber(
        "Initial Intake Motor Goal Speed (Rotations/Second)", goalPIDInitialIntakeSpeedRPS());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Goal Speed (Rotations/Second)", goalPIDKickerIntakeSpeedRPS());
    SmartDashboard.putNumber(
        "Initial Intake Motor Speed Error", mInitialIntake.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Speed Error", mKickerIntake.getClosedLoopError().getValueAsDouble());
  }
}
