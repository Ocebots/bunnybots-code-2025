package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.CANMappings;
import frc.robot.config.PivotConfig;

public class Pivot {
    protected TalonFX mPivotLeft;
    protected TalonFX mPivotRight;
    protected Follower follower;
    private double currentAngleDegrees;

    public Pivot() {
        mPivotLeft=new TalonFX(CANMappings.PIVOT_LEFT_ID);
        mPivotRight=new TalonFX(CANMappings.PIVOT_RIGHT_ID);

        TalonFXConfiguration leftPivotConfig=new TalonFXConfiguration();
        TalonFXConfiguration rightPivotConfig=new TalonFXConfiguration();

        leftPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftPivotConfig.CurrentLimits.SupplyCurrentLimit= PivotConfig.PIVOT_SUPPLY_CURRENT_LIMIT;
        rightPivotConfig.CurrentLimits.SupplyCurrentLimit= PivotConfig.PIVOT_STATOR_CURRENT_LIMIT;

        leftPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftPivotConfig.CurrentLimits.StatorCurrentLimit= PivotConfig.PIVOT_SUPPLY_CURRENT_LIMIT;
        rightPivotConfig.CurrentLimits.StatorCurrentLimit= PivotConfig.PIVOT_STATOR_CURRENT_LIMIT;

        leftPivotConfig.MotionMagic.MotionMagicCruiseVelocity= PivotConfig.PIVOT_MAX_CRUISE_VELOCITY;
        leftPivotConfig.MotionMagic.MotionMagicAcceleration= PivotConfig.PIVOT_TARGET_ACCELERATION;

        rightPivotConfig.MotionMagic.MotionMagicCruiseVelocity= PivotConfig.PIVOT_MAX_CRUISE_VELOCITY;
        rightPivotConfig.MotionMagic.MotionMagicAcceleration= PivotConfig.PIVOT_TARGET_ACCELERATION;

        leftPivotConfig.Slot0.kP=PivotConfig.PIVOT_P;
        leftPivotConfig.Slot0.kI=PivotConfig.PIVOT_I;
        leftPivotConfig.Slot0.kD=PivotConfig.PIVOT_D;
        leftPivotConfig.Slot0.kS=PivotConfig.PIVOT_S;
        leftPivotConfig.Slot0.kV=PivotConfig.PIVOT_G;
        leftPivotConfig.Slot0.kG=PivotConfig.PIVOT_V;
        leftPivotConfig.Slot0.kA=PivotConfig.PIVOT_A;

        rightPivotConfig.Slot0.kP=PivotConfig.PIVOT_P;
        rightPivotConfig.Slot0.kI=PivotConfig.PIVOT_I;
        rightPivotConfig.Slot0.kD=PivotConfig.PIVOT_D;
        rightPivotConfig.Slot0.kS=PivotConfig.PIVOT_S;
        rightPivotConfig.Slot0.kV=PivotConfig.PIVOT_G;
        rightPivotConfig.Slot0.kG=PivotConfig.PIVOT_V;
        rightPivotConfig.Slot0.kA=PivotConfig.PIVOT_A;

        leftPivotConfig.Feedback.SensorToMechanismRatio=PivotConfig.LEFT_PIVOT_GEAR_RATIO;
        rightPivotConfig.Feedback.SensorToMechanismRatio=PivotConfig.RIGHT_PIVOT_GEAR_RATIO;

        leftPivotConfig.MotorOutput.NeutralMode= NeutralModeValue.Coast;
        rightPivotConfig.MotorOutput.NeutralMode= NeutralModeValue.Coast;

        mPivotLeft.getConfigurator().apply(leftPivotConfig);
        mPivotRight.getConfigurator().apply(rightPivotConfig);

        follower=new Follower(CANMappings.PIVOT_LEFT_ID, true);
    }

    public void setPivotAngle(Rotation2d angle){
        mPivotRight.setControl(new MotionMagicVoltage(angle.getRotations()));
        mPivotLeft.setControl(follower);
    }

    public Rotation2d getHighShootAngle(){
        return Rotation2d.fromDegrees(0.0);
    }
    public Rotation2d getLowShootAngle(){
        return Rotation2d.fromDegrees(0.0);
    }
    public void zeroPivot(){
        mPivotLeft.setPosition(0.0);
        mPivotRight.setPosition(0.0);
    }
    public void stopPivot(){
        mPivotLeft.set(0.0);
        mPivotRight.set(0.0);
    }
    public double getPivotAngleRotations(){
        currentAngleDegrees=mPivotLeft.getPosition().getValueAsDouble();
        return currentAngleDegrees;
    }
    public double getPivotAngleDegrees(){
        currentAngleDegrees=mPivotLeft.getPosition().getValueAsDouble() * 360;
        return currentAngleDegrees%360;
    }
}
