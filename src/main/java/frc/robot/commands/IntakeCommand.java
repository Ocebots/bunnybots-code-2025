package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.IntakeConfig;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    public static enum Positions {
        INTAKE,
        OUTTAKE,
        SHOOT
    }

    private Positions pose;
    Intake intake;

    public IntakeCommand(Intake intake, Positions pose) {
        this.intake = intake;
        this.pose = pose;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        switch (pose) {
            case INTAKE:
                intake.runKicker(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
                intake.runInitial(IntakeConfig.K_INITIAL_INTAKE_VELOCITY);
                break;

            case OUTTAKE:
                intake.runKicker(IntakeConfig.K_KICKER_OUTTAKE_VELOCITY);
                intake.runInitial(IntakeConfig.K_INITIAL_OUTTAKE_VELOCITY);
                break;

            case SHOOT:
                intake.runKicker(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
                intake.runInitial(IntakeConfig.K_KICKER_INTAKE_VELOCITY);
                break;

            default:
                intake.stopKicker();
                intake.stopInitial();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
