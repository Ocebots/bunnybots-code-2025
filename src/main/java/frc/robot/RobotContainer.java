// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.CANMappings;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Intake intake = new Intake();
  private Pivot pivot = new Pivot();
  private Shooter shooter = new Shooter();
  private final Superstructure superstructure =
      new Superstructure(intake, pivot, shooter, drivetrain);
  private final Pigeon2 pigeon2 = new Pigeon2(CANMappings.PIGEON_CAN_ID);

  // links xbox controller to controls
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.leftTrigger().onTrue(superstructure.toggleCloseHigh());
    controller.rightTrigger().onTrue(superstructure.action());
    controller.leftBumper().onTrue(superstructure.toggleFarHigh());
    controller.rightBumper().onTrue(superstructure.toggleLowScore());
    controller.rightStick().onTrue(superstructure.toggleIntake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
