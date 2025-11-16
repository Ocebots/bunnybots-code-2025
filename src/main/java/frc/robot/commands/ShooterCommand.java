package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  public static enum Positions {
    SHOOT,
    IDLE
  }

  private Positions pose;
  Shooter shooter;

  public ShooterCommand(Shooter shooter, Positions pose) {
    this.shooter = shooter;
    this.pose = pose;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case SHOOT:
        shooter.shoot(ShooterConfig.K_TOP_AND_BOTTOM_SHOOTER_VELOCITY);
        break;

      case IDLE:
        shooter.stopShooter();
        break;

      default:
        shooter.stopShooter();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }
}
