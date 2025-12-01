package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.config.CANMappings;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

@Logged
public class Superstructure {
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  private SuperState state = SuperState.IDLE;

  public Superstructure(
      Intake intake, Pivot pivot, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
  }

  public enum SuperState {
    IDLE,
    READY_CLOSE_HIGH,
    READY_FAR_HIGH,
    READY_LOW_SCORE,
    INTAKE
  }

  public Command toggleCloseHigh() {
    return Commands.runOnce(
        () -> {
          if (state == SuperState.READY_CLOSE_HIGH) {
            setState(SuperState.IDLE);
          } else {
            setState(SuperState.READY_CLOSE_HIGH);
          }
        });
  }

  public Command toggleFarHigh() {
    return Commands.runOnce(
        () -> {
          if (state == SuperState.READY_FAR_HIGH) {
            setState(SuperState.IDLE);
          } else {
            setState(SuperState.READY_FAR_HIGH);
          }
        });
  }

  public Command toggleLowScore() {
    return Commands.runOnce(
        () -> {
          if (state == SuperState.READY_LOW_SCORE) {
            setState(SuperState.IDLE);
          } else {
            setState(SuperState.READY_LOW_SCORE);
          }
        });
  }

  public Command toggleIntake() {
    return Commands.runOnce(
        () -> {
          if (state == SuperState.INTAKE) {
            setState(SuperState.IDLE);
          } else {
            setState(SuperState.INTAKE);
          }
        });
  }

  public Command action() {
    return Commands.runOnce(
        () -> {
          switch (state) {
            case READY_CLOSE_HIGH:
            case READY_FAR_HIGH:
              new ShooterCommand(shooter, ShooterCommand.Positions.SHOOT);
            case READY_LOW_SCORE:
              new IntakeCommand(intake, IntakeCommand.Speeds.OUTTAKE_SCORE);
            case INTAKE:
              new IntakeCommand(intake, IntakeCommand.Speeds.OUTTAKE_SCORE);
            case IDLE:
              break;
          }
        });
  }

  private void setState(SuperState newState) {
    state = newState;

    switch (newState) {
      case IDLE:
        new DrivetrainCommand(
            drivetrain, DrivetrainCommand.Position.IDLE, new CommandXboxController(0));
        new PivotCommand(pivot, PivotCommand.Position.IDLE);
        new IntakeCommand(intake, IntakeCommand.Speeds.IDLE);
        new ShooterCommand(shooter, ShooterCommand.Positions.IDLE);
      case READY_CLOSE_HIGH:
        new DrivetrainCommand(
            drivetrain, pivot.getClosestCosmicConverterDrivetrain(), new CommandXboxController(0));
        new PivotCommand(pivot, pivot.getClosestCosmicConverterPivot());
        new IntakeCommand(intake, IntakeCommand.Speeds.IDLE);
        new ShooterCommand(shooter, ShooterCommand.Positions.IDLE);
      case READY_FAR_HIGH:
        new DrivetrainCommand(
            drivetrain, pivot.getFarthestCosmicConverterDrivetrain(), new CommandXboxController(0));
        new PivotCommand(pivot, pivot.getFarthestCosmicConverterPivot());
        new IntakeCommand(intake, IntakeCommand.Speeds.IDLE);
        new ShooterCommand(shooter, ShooterCommand.Positions.IDLE);
      case READY_LOW_SCORE:
        new DrivetrainCommand(
            drivetrain, DrivetrainCommand.Position.IDLE, new CommandXboxController(0));
        new PivotCommand(pivot, PivotCommand.Position.OUTTAKE_SCORE);
        new IntakeCommand(intake, IntakeCommand.Speeds.IDLE);
        new ShooterCommand(shooter, ShooterCommand.Positions.IDLE);
      case INTAKE:
        new DrivetrainCommand(
            drivetrain, DrivetrainCommand.Position.IDLE, new CommandXboxController(0));
        new PivotCommand(pivot, PivotCommand.Position.INTAKE_GROUND);
        new IntakeCommand(intake, IntakeCommand.Speeds.INTAKE);
        new ShooterCommand(shooter, ShooterCommand.Positions.IDLE);
    }
  }

  public static void zeroPigeon() {
    Pigeon2 pigeon = new Pigeon2(CANMappings.PIGEON_CAN_ID);
    pigeon.reset();
  }
}
