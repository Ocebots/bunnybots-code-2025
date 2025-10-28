package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

public class Intake {
  protected TalonFX mInitialIntake;
  protected TalonFX mKickerIntake;

  public Intake() {
    mInitialIntake = new TalonFX(CANMappings.K_INITIAL_INTAKE_ID);
    mKickerIntake = new TalonFX(CANMappings.K_KICKER_INTAKE_ID);

    TalonFXConfiguration initialIntakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerIntakeConfig = new TalonFXConfiguration();

    initialIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    initialIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.K_INTAKE_MOTORS_SUPPLY_CURRENT_LIMIT;
    initialIntakeConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConfig.K_INTAKE_MOTORS_SUPPLY_CURRENT_LIMIT;
  }
}
