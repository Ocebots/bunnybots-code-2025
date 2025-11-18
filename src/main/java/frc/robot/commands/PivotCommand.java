package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.PivotConfig;
import frc.robot.subsystems.Pivot;

public class PivotCommand extends Command {
  public static enum Positions {
    INTAKE_GROUND,
    INTAKE_STAR_SPIRE,
    OUTTAKE_SCORE,
    INNER_HIGH_SHOOT,
    OUTER_HIGH_SHOOT,
    IDLE
  }

  private Positions pose;
  Pivot pivot;

  public PivotCommand(Pivot pivot, Positions pose) {
    this.pivot = pivot;
    this.pose = pose;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case INTAKE_GROUND:
        pivot.setPivotAngle(
            new Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_GROUND_INTAKE_ANGLE)));
        break;

      case INTAKE_STAR_SPIRE:
        pivot.setPivotAngle(
            new Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_STAR_SPIRE_INTAKE_ANGLE)));
        break;

      case OUTTAKE_SCORE:
        pivot.setPivotAngle(
            new Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_OUTTAKE_ANGLE)));
        break;

      case IDLE:
        pivot.setPivotAngle(new Rotation2d((Units.degreesToRadians(PivotConfig.PIVOT_IDLE_ANGLE))));
        break;

      case INNER_HIGH_SHOOT:
        pivot.setPivotAngle(pivot.getHighAngle(Pivot.getLocation(1)));
        break;

      case OUTER_HIGH_SHOOT:
        pivot.setPivotAngle(pivot.getHighAngle(Pivot.getLocation(0)));
        break;

      default:
        pivot.stopPivot();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    // The command is finished when the pivot reaches its target angle within tolerance.
    return pivot.pivotAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stopPivot();
  }
}
