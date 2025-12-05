package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.PivotConfig;
import frc.robot.config.TunerConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;

@Logged
public class Pivot extends SubsystemBase {
    protected TalonFX mPivotLeft;
    protected TalonFX mPivotRight;
    protected Follower follower;
    protected CommandSwerveDrivetrain drivetrain;
    protected Shooter shooter;
    protected Intake intake;
    private final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

    private double currentAngle;

    public Pivot() {
        mPivotLeft = new TalonFX(CANMappings.K_PIVOT_LEFT_ID);
        mPivotRight = new TalonFX(CANMappings.K_PIVOT_RIGHT_ID);

        drivetrain = TunerConstants.createDrivetrain();
        shooter = new Shooter();
        intake = new Intake();
        TalonFXConfiguration leftPivotConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightPivotConfig = new TalonFXConfiguration();

        leftPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftPivotConfig.CurrentLimits.StatorCurrentLimit =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_STATOR_CURRENT_LIMIT;
        leftPivotConfig.CurrentLimits.SupplyCurrentLimit =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_SUPPLY_CURRENT_LIMIT;
        ;

        rightPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightPivotConfig.CurrentLimits.StatorCurrentLimit =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_STATOR_CURRENT_LIMIT;
        ;
        rightPivotConfig.CurrentLimits.SupplyCurrentLimit =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_SUPPLY_CURRENT_LIMIT;
        ;

        leftPivotConfig.MotionMagic.MotionMagicCruiseVelocity =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_MAX_CRUISE_VELOCITY;
        rightPivotConfig.MotionMagic.MotionMagicCruiseVelocity =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_MAX_CRUISE_VELOCITY;
        leftPivotConfig.MotionMagic.MotionMagicAcceleration =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_TARGET_ACCELERATION;
        rightPivotConfig.MotionMagic.MotionMagicAcceleration =
                PivotConfig.K_LEFT_AND_RIGHT_PIVOT_TARGET_ACCELERATION;

        leftPivotConfig.Slot0.kP = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_P;
        leftPivotConfig.Slot0.kI = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_I;
        leftPivotConfig.Slot0.kD = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_D;
        leftPivotConfig.Slot0.kS = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_S;
        leftPivotConfig.Slot0.kG = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_G;
        leftPivotConfig.Slot0.kV = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_V;
        leftPivotConfig.Slot0.kA = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_A;

        rightPivotConfig.Slot0.kP = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_P;
        rightPivotConfig.Slot0.kI = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_I;
        rightPivotConfig.Slot0.kD = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_D;
        rightPivotConfig.Slot0.kS = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_S;
        rightPivotConfig.Slot0.kG = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_G;
        rightPivotConfig.Slot0.kV = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_V;
        rightPivotConfig.Slot0.kA = PivotConfig.K_LEFT_AND_RIGHT_PIVOT_A;

        leftPivotConfig.Feedback.SensorToMechanismRatio =
                PivotConfig.K_LEFT_PIVOT_GEAR_RATIO; // gear ratio
        rightPivotConfig.Feedback.SensorToMechanismRatio = PivotConfig.K_RIGHT_PIVOT_GEAR_RATIO;

        leftPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // leftPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mPivotLeft.getConfigurator().apply(leftPivotConfig);
        mPivotRight.getConfigurator().apply(rightPivotConfig);

        // follower = new Follower(CANMappings.K_PIVOT_LEFT_ID, false);
    }

    public void setPivotAngle(Rotation2d angleSetpoint) {
        mPivotLeft.setControl(new MotionMagicVoltage(angleSetpoint.getRotations()));
        mPivotRight.setControl(follower);
    }

    public void setPivotAngleRot(double rotation) {
        mPivotLeft.setControl(new MotionMagicVoltage(-rotation));
        mPivotRight.setControl(new MotionMagicVoltage(rotation));
    }

    public void pivotDefault() {
        mPivotLeft.setControl(new MotionMagicVoltage(0.0));
        mPivotRight.setControl(new MotionMagicVoltage(0.0));
    }

    public void zeroPivot() {
        mPivotLeft.setPosition(0.0);
        mPivotRight.setPosition(0.0);
    }

    public void stopPivot() {
        mPivotLeft.stopMotor();
        mPivotRight.stopMotor();
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(mPivotLeft.getClosedLoopError().getValueAsDouble())
                <= PivotConfig.K_PIVOT_ANGLE_TOLERANCE;
    }

    public double getPivotAngleDegrees() {
        currentAngle = mPivotLeft.getPosition().getValueAsDouble();
        currentAngle = currentAngle * 360;

        return currentAngle;
    }

    private final SwerveRequest.FieldCentricFacingAngle m_faceAngle =
            new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(
                            SwerveModule.DriveRequestType.OpenLoopVoltage); // Or OpenLoopDutyCycle

    public Command getCosmicConverter(BooleanSupplier complete, boolean isInner) {
        Optional<DriverStation.Alliance> alliance1 = DriverStation.getAlliance();
        Translation2d cosmicConverter = new Translation2d();
        map.put(Units.inchesToMeters(59.0), 0.18);
        map.put(Units.inchesToMeters(76.5), 0.155);
        map.put(Units.inchesToMeters(96.5), 0.142);
        map.put(Units.inchesToMeters(125.5), 0.13);
        map.put(Units.inchesToMeters(169.5), 0.12);
        map.put(Units.inchesToMeters(210.5), 0.118);
        if (alliance1.isPresent()) {
            if (alliance1.get() == DriverStation.Alliance.Blue) {
                if (isInner) {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(196.125));
                } else {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(20.5));
                }
            }
            if (alliance1.get() == DriverStation.Alliance.Red) {
                if (isInner) {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(196.125));
                } else {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(20.5));
                }
            }

            Rotation2d heading = drivetrain.getState().Pose.getRotation();

            // shooter offset in robot frame (meters)
            double shooterOffsetX = 0.0; // forward
            double shooterOffsetY = Units.inchesToMeters(-1); // right

            // convert to field frame
            double shooterX =
                    drivetrain.getState().Pose.getX()
                            + shooterOffsetX * heading.getCos()
                            - shooterOffsetY * heading.getSin();

            double shooterY =
                    drivetrain.getState().Pose.getY()
                            + shooterOffsetX * heading.getSin()
                            + shooterOffsetY * heading.getCos();

            // compute target angle
            Rotation2d aimAngle =
                    new Rotation2d(
                            Math.atan2(
                                    cosmicConverter.getY() - drivetrain.getState().Pose.getY(),
                                    cosmicConverter.getX() - drivetrain.getState().Pose.getX()));

            return Commands.runOnce(
                            () -> drivetrain.setControl(m_faceAngle.withTargetDirection(aimAngle)), drivetrain)
                    .alongWith(
                            Commands.runOnce(
                                    () ->
                                            setPivotAngleRot(
                                                    map.get(
                                                            drivetrain
                                                                    .getState()
                                                                    .Pose
                                                                    .getTranslation()
                                                                    .getDistance(getCosmicConverterTranslation(false))))))
                    .andThen(
                            Commands.waitUntil(complete)
                                    .andThen(
                                            Commands.parallel(
                                                            Commands.run((() -> intake.runKicker(-0.7)))
                                                                    .withTimeout(0.5)
                                                                    .andThen(
                                                                            Commands.run(() -> shooter.shoot())
                                                                                    .alongWith(Commands.run(() -> intake.intake()))))
                                                    .until(() -> !complete.getAsBoolean())));
        } else {
            cosmicConverter = null;
            System.out.println("no alliance detected: likely causing many errors");
            return null;
        }
    }

    public Translation2d getCosmicConverterTranslation(boolean isInner) {
        Optional<DriverStation.Alliance> alliance1 = DriverStation.getAlliance();
        Translation2d cosmicConverter = new Translation2d();
        if (alliance1.isPresent()) {
            if (alliance1.get() == DriverStation.Alliance.Blue) {
                if (isInner) {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(196.125));
                } else {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(4.0), Units.inchesToMeters(20.5));
                }
            }
            if (alliance1.get() == DriverStation.Alliance.Red) {
                if (isInner) {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(196.125));
                } else {
                    cosmicConverter =
                            new Translation2d(Units.inchesToMeters(644.0), Units.inchesToMeters(20.5));
                }
            }
        }
        return cosmicConverter;
    }
    public Command defaults(){
        return Commands.run(()->drivetrain.setControl(m_faceAngle.withTargetDirection(drivetrain.getState().Pose.getRotation())));
    }
    public Command lowScore(double angle) {
        return Commands.run(() -> setPivotAngleRot(angle));
    }
}
