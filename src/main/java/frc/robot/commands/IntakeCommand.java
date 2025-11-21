package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.IntakeConfig;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  public static enum Speeds {
    INTAKE,
    OUTTAKE_SCORE,
    SHOOT,
    IDLE
  }

  private Speeds speed;
  Intake intake;

  public IntakeCommand(Intake intake, Speeds speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    switch (speed) {
      case INTAKE:
        intake.runKicker(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
        intake.runInitial(IntakeConfig.K_INITIAL_INTAKE_VELOCITY);
        break;

      case OUTTAKE_SCORE:
        intake.runKicker(IntakeConfig.K_KICKER_OUTTAKE_VELOCITY);
        intake.runInitial(IntakeConfig.K_INITIAL_OUTTAKE_VELOCITY);
        break;

      case SHOOT:
        intake.runKicker(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
        intake.runInitial(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
        break;

      case IDLE:
        intake.stopIntake();
        break;

      default:
        intake.stopIntake();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }
}
