package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommand extends Command {
  /*public static enum Positions {
      INTAKE,
      OUTTAKE,
      HIGH_SHOOT,
      LOW_SHOOT,
      IDLE
  }
  private Positions pose;
  Pivot pivot;
  Intake intake;

  public PivotCommand(Pivot pivot, Intake intake, Positions pose) {
      this.pivot = pivot;
      this.pose = pose;
      this.intake = intake;
      addRequirements(pivot, intake);
  }

  @Override
  public void initialize() {
      switch (pose) {
          case INTAKE:
              pivot.setPivotAngle(new Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_INTAKE_ANGLE)));
              intake.intake(0.0);
              break;

          case OUTTAKE:
              pivot.setPivotAngle(new Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_INTAKE_ANGLE)));
              intake.intake(-0.0);
              break;

          case IDLE:
              pivot.setPivotAngle(new Rotation2d((Units.degreesToRadians(PivotConfig.PIVOT_IDLE_ANGLE))));
              intake.stopIntake();
              break;

          case HIGH_SHOOT:
              pivot.setPivotAngle(pivot.getHighShooterAngle());
              intake.stopIntake();
              break;

          case LOW_SHOOT:
              pivot.setPivot(pivot.getLowShooterAngle());
              intake.stopIntake();
              break;

          default:
              pivot.stopPivot();
              intake.stopIntake();
              break;
      }

  }

  @Override
  public void end(boolean interrupted) {
      pivot.stopPivot();
      intake.stopIntake();
  }
  */
}
