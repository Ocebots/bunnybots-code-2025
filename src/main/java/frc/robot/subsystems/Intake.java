package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.CANMappings;

public class Intake {
  protected TalonFX mInitialIntake;
  protected TalonFX mKickerIntake;

  private double tolerance;
  private double setpointInitialIntake;
  private double setpointKickerIntake;

  public Intake() {
    mInitialIntake = new TalonFX(CANMappings.K_INITIAL_INTAKE_ID);
    mKickerIntake = new TalonFX(CANMappings.K_KICKER_INTAKE_ID);

    TalonFXConfiguration initialIntakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerIntakeConfig = new TalonFXConfiguration();

    initialIntakeConfig.Slot0.kP = 0.0;
    initialIntakeConfig.Slot0.kI = 0.0;
    initialIntakeConfig.Slot0.kD = 0.0;
    initialIntakeConfig.Slot0.kS = 0.0;
    initialIntakeConfig.Slot0.kV = 0.0;
    initialIntakeConfig.Slot0.kA = 0.0;

    kickerIntakeConfig.Slot0.kP = 0.0;
    kickerIntakeConfig.Slot0.kI = 0.0;
    kickerIntakeConfig.Slot0.kD = 0.0;
    kickerIntakeConfig.Slot0.kS = 0.0;
    kickerIntakeConfig.Slot0.kV = 0.0;
    kickerIntakeConfig.Slot0.kA = 0.0;

    initialIntakeConfig.Feedback.SensorToMechanismRatio = 25; // gear ratio
    kickerIntakeConfig.Feedback.SensorToMechanismRatio = 25;

    initialIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mInitialIntake.getConfigurator().apply(initialIntakeConfig);
    mKickerIntake.getConfigurator().apply(kickerIntakeConfig);
  }

  // Velocity is rotations per second of motor accounting for SensorToMechanismRatio
  public void intake(double velocity) {
    setpointInitialIntake = velocity;
    setpointKickerIntake = velocity;
    tolerance = velocity / 10;
    mInitialIntake.setControl(new VelocityVoltage(velocity));
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  public void stopIntake() {
    tolerance = 0;
    setpointInitialIntake = 0;
    setpointKickerIntake = 0;
    mInitialIntake.stopMotor();
    mKickerIntake.stopMotor();
  }

  public void runKicker(double velocity) {
    setpointKickerIntake = velocity;
    mKickerIntake.setControl(new VelocityVoltage(velocity));
  }

  public boolean intakeActive() {
    return Math.abs(mInitialIntake.getVelocity().getValueAsDouble()) > 0
        || Math.abs(mKickerIntake.getVelocity().getValueAsDouble()) > 0;
  }

  public double actualInitialIntakeMotorSpeedRPS() {
    return mInitialIntake.getVelocity().getValueAsDouble();
  }

  public double actualKickerIntakeMotorSpeedRPS() {
    return mKickerIntake.getVelocity().getValueAsDouble();
  }

  public double goalIntakeSpeedRPS() {
    return mInitialIntake.getClosedLoopReference().getValueAsDouble();
  }

  public boolean initialIntakeMotorAtGoalSpeed() {
    return Math.abs(actualInitialIntakeMotorSpeedRPS() - setpointInitialIntake) <= tolerance;
  }

  public boolean kickerIntakeMotorAtGoalSpeed() {
    return Math.abs(actualKickerIntakeMotorSpeedRPS() - setpointKickerIntake) <= tolerance;
  }

  // @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Active", intakeActive());
    SmartDashboard.putNumber(
        "Initial Intake Motor Actual Speed (Rotations/Second)", actualInitialIntakeMotorSpeedRPS());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Actual Speed (Rotations/Second)", actualKickerIntakeMotorSpeedRPS());
    SmartDashboard.putNumber("Intake Goal Speed (Rotations/Second)", goalIntakeSpeedRPS());
    SmartDashboard.putBoolean(
        "Initial Intake Motor Is At Goal Speed", initialIntakeMotorAtGoalSpeed());
    SmartDashboard.putBoolean(
        "Kicker Intake Motor Is At Goal Speed", kickerIntakeMotorAtGoalSpeed());
    SmartDashboard.putNumber(
        "Initial Intake Motor Speed Error", mInitialIntake.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Speed Error", mKickerIntake.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Initial Intake Motor Output %", mInitialIntake.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker Intake Motor Output %", mKickerIntake.getMotorOutputStatus().getValueAsDouble());
  }
}
