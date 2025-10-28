package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.PivotConfig;


public class Pivot extends SubsystemBase {
    protected TalonFX pivotRightMotor;
    protected TalonFX pivotLeftMotor;
    protected Follower follower;

    public Pivot() {
        pivotRightMotor = new TalonFX(CANMappings.PIVOT_RIGHT_ID);
        pivotLeftMotor = new TalonFX(CANMappings.PIVOT_LEFT_ID);
        follower = new Follower(pivotLeftMotor.getDeviceID(), true);

        TalonFXConfiguration pivotLeftConfig = new TalonFXConfiguration();
        TalonFXConfiguration pivotRightConfig = new TalonFXConfiguration();

        pivotRightConfig.MotorOutput.Inverted = pivotRightConfig.MotorOutput.Inverted.Clockwise_Positive;

        Slot0Configs pivotRightSlot0 = pivotRightConfig.Slot0;
        pivotRightSlot0.kS = PivotConfig.PIVOT_S;
        pivotRightSlot0.kG = PivotConfig.PIVOT_G;
        pivotRightSlot0.kV = PivotConfig.PIVOT_V;
        pivotRightSlot0.kA = PivotConfig.PIVOT_A;
        pivotRightSlot0.kP = PivotConfig.PIVOT_P;
        pivotRightSlot0.kI = PivotConfig.PIVOT_I;
        pivotRightSlot0.kD = PivotConfig.PIVOT_D;

        Slot0Configs pivotLeftSlot0 = pivotLeftConfig.Slot0;
        pivotLeftSlot0.kS = PivotConfig.PIVOT_S;
        pivotLeftSlot0.kV = PivotConfig.PIVOT_V;
        pivotLeftSlot0.kG = PivotConfig.PIVOT_G;
        pivotLeftSlot0.kA = PivotConfig.PIVOT_A;
        pivotLeftSlot0.kP = PivotConfig.PIVOT_P;
        pivotLeftSlot0.kI = PivotConfig.PIVOT_I;
        pivotLeftSlot0.kD = PivotConfig.PIVOT_D;

        pivotRightMotor.getConfigurator().apply(pivotRightConfig);
        pivotLeftMotor.getConfigurator().apply(pivotLeftConfig);

        pivotRightMotor.setControl(follower);
    }

    public void setPivot(Rotation2d angle) {
        pivotRightMotor.setControl(new MotionMagicVoltage(angle.getDegrees()));
        pivotLeftMotor.setControl(follower);
    }

    public Rotation2d getHighShooterAngle() {
        // This will do math in the future to figure out the angle to shoot based on distance
         return new Rotation2d(Units.degreesToRadians(50));
    }

    public Rotation2d getLowShooterAngle() {
        // This will do math in the future to figure out the angle to shoot based on distance
        return new Rotation2d(Units.degreesToRadians(50));
    }

    public void stopPivot() {
        pivotRightMotor.set(0);
        pivotLeftMotor.set(0);
    }

    public double getPivotAngle() {
        return pivotLeftMotor.getPosition().getValueAsDouble();
    }
}
