package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@CustomLoggerFor(CommandSwerveDrivetrain.class)
public class CommandSwerveDrivetrainLogger extends ClassSpecificLogger<CommandSwerveDrivetrain> {
  public CommandSwerveDrivetrainLogger() {
    super(CommandSwerveDrivetrain.class);
  }

  @Override
  protected void update(EpilogueBackend backend, CommandSwerveDrivetrain drivetrain) {
    // backend.log(drivetrain.getState().ModulePositions.);
  }
}
