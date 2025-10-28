package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

public class Intake extends SubsystemBase {
   protected TalonFX intakeMotor;

   private SimpleMotorFeedforward feedforward;

   public Intake() {
       intakeMotor = new TalonFX(CANMappings.INTAKE_ID);

       feedforward = new SimpleMotorFeedforward(IntakeConfig.INTAKE_S, IntakeConfig.INTAKE_V);
   }

   public void intake() {
        intakeMotor.setControl(new DutyCycleOut(feedforward.calculate(IntakeConfig.INTAKE_VELOCITY)));
   }

   public void outtake() {
       intakeMotor.setControl(new DutyCycleOut(-feedforward.calculate(IntakeConfig.INTAKE_VELOCITY)));
   }

   public void stopIntake() {
       intakeMotor.set(0);
   }
}
