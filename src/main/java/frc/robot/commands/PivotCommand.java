package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.PivotConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PivotCommand extends Command {
    public static enum Positions {
      GROUND_INTAKE,
        STAR_SPIRE_INTAKE,
      OUTTAKE,
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
        case GROUND_INTAKE:
          pivot.setPivotAngle(new
   Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_GROUND_INTAKE_ANGLE)));
          break;

          case STAR_SPIRE_INTAKE:
              pivot.setPivotAngle(new
                      Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_STAR_SPIRE_INTAKE_ANGLE)));
              break;

        case OUTTAKE:
          pivot.setPivotAngle(new
   Rotation2d(Units.degreesToRadians(PivotConfig.PIVOT_OUTTAKE_ANGLE)));
          break;
  
        case IDLE:
          pivot.setPivotAngle(new
   Rotation2d((Units.degreesToRadians(PivotConfig.PIVOT_IDLE_ANGLE))));
          break;
  
        case INNER_HIGH_SHOOT:
            int innerLocation = 0;
            if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
                innerLocation = 1;
            }
            if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
                innerLocation = 3;
            }
          pivot.setPivotAngle(pivot.getHighAngle(innerLocation));
          break;

          case OUTER_HIGH_SHOOT:
              int outerLocation = 0;
              if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
                  outerLocation = 2;
          }
              if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
                  outerLocation = 4;
              }
              pivot.setPivotAngle(pivot.getHighAngle(outerLocation));
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
