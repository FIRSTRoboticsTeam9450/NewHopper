package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Flywheel + hood, driven entirely by manual driver adjustment.
 *
 * <p>There is no distance/vision based aiming here: the flywheel velocity and the hood position are
 * both operator-nudged setpoints that stay where they are put, so the turret never chases a computed
 * angle. Ported from the previous robot's ShooterSubsystem, minus the interpolation tables and the
 * zone based auto aim.
 */
public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem INSTANCE;

    public enum AngleState {
        SHOOT,
        STORE
    }

    public enum ShooterState {
        SHOOT,
        IDLE
    }

    private AngleState angleState = AngleState.STORE;
    private ShooterState shooterState = ShooterState.IDLE;

    /* ========== HOOD (MANUAL) ========== */
    // TODO: Re-measure on this robot, the previous bot's hood travel was 14.3 revs
    public static final double kHoodMinPosition = 0; // Motor revolutions, hood fully stored
    public static final double kHoodMaxPosition = 14.3; // Motor revolutions, hood fully raised
    public static final double kHoodStep = 0.5; // Motor revolutions moved per button press

    /* ========== FLYWHEEL (MANUAL) ========== */
    // TODO: Re-measure on this robot, the previous bot topped out around 100 rps
    public static final double kShooterMinVelocity = 0; // Rotations per second
    public static final double kShooterMaxVelocity = 100; // Rotations per second
    public static final double kShooterStep = 2.5; // Rotations per second added per button press
    public static final double kShooterDefaultVelocity = 20; // Rotations per second, what a reset goes back to

    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterLeftMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterRightMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning, defaults are carried over from the previous robot and will need a retune
    private LoggedNetworkNumber logAngleAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Accel", 800);
    private LoggedNetworkNumber logAngleVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Veloc", 800);
    private LoggedNetworkNumber logAngleKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", 0.1);
    private LoggedNetworkNumber logAngleKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.095);
    private LoggedNetworkNumber logAngleKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0);
    private LoggedNetworkNumber logAngleKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 15);
    private LoggedNetworkNumber logAngleKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
    private LoggedNetworkNumber logAngleKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
    private LoggedNetworkNumber logAngleKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.3);
    private LoggedNetworkNumber logShooterAccel = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/Accel", 9450);
    private LoggedNetworkNumber logShooterKS = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kS", 0.023549);
    private LoggedNetworkNumber logShooterKV = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kV", 0.11935);
    private LoggedNetworkNumber logShooterKA = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kA", 0.025517);
    private LoggedNetworkNumber logShooterKP = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kP", 0.4);
    private LoggedNetworkNumber logShooterKI = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kI", 0);
    private LoggedNetworkNumber logShooterKD = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kD", 0);

    private MotionMagicVoltage angleRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private MotionMagicVelocityVoltage shootRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logShooterDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    private double angleSetpoint = 0;
    private double shooterSetpoint = 0;

    /** Manually adjusted hood position, in motor revolutions. */
    private double hoodTarget = kHoodMinPosition;
    /** Manually adjusted flywheel velocity, in rotations per second. */
    private double shooterTarget = kShooterDefaultVelocity;

    private ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();
    }

    private void configureAngleMotors() {
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Up, Negative = Down

        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kHoodMaxPosition;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        angleMotorConfig.MotionMagic.MotionMagicAcceleration = logAngleAccel.get();
        angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = logAngleVeloc.get();

        angleMotorConfig.Slot0.kS = logAngleKS.get();
        angleMotorConfig.Slot0.kV = logAngleKV.get();
        angleMotorConfig.Slot0.kA = logAngleKA.get();
        angleMotorConfig.Slot0.kP = logAngleKP.get();
        angleMotorConfig.Slot0.kI = logAngleKI.get();
        angleMotorConfig.Slot0.kD = logAngleKD.get();
        angleMotorConfig.Slot0.kG = logAngleKG.get();
        angleMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        angleMotor.getConfigurator().apply(angleMotorConfig);
    }

    private void configureShooterMotors() {
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Outtake, Negative = Intake

        shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;

        shooterMotorConfig.MotionMagic.MotionMagicAcceleration = logShooterAccel.get();
        // No CruiseVelocity because it's set to run to a specific velocity

        shooterMotorConfig.Slot0.kS = logShooterKS.get();
        shooterMotorConfig.Slot0.kV = logShooterKV.get();
        shooterMotorConfig.Slot0.kA = logShooterKA.get();
        shooterMotorConfig.Slot0.kP = logShooterKP.get();
        shooterMotorConfig.Slot0.kI = logShooterKI.get();
        shooterMotorConfig.Slot0.kD = logShooterKD.get();
        // No kG or GravityType because it spins freely and does not have gravity

        shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
        shooterRightMotor.getConfigurator().apply(shooterMotorConfig);

        shooterRightMotor.setControl(new Follower(shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void addData() {
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    public void updateConstants() {
        if (
            angleMotorConfig.MotionMagic.MotionMagicAcceleration != logAngleAccel.get() ||
            angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logAngleVeloc.get() ||
            angleMotorConfig.Slot0.kS != logAngleKS.get() ||
            angleMotorConfig.Slot0.kV != logAngleKV.get() ||
            angleMotorConfig.Slot0.kA != logAngleKA.get() ||
            angleMotorConfig.Slot0.kP != logAngleKP.get() ||
            angleMotorConfig.Slot0.kI != logAngleKI.get() ||
            angleMotorConfig.Slot0.kD != logAngleKD.get() ||
            angleMotorConfig.Slot0.kG != logAngleKG.get()
        ) configureAngleMotors();

        if (
            shooterMotorConfig.MotionMagic.MotionMagicAcceleration != logShooterAccel.get() ||
            shooterMotorConfig.Slot0.kS != logShooterKS.get() ||
            shooterMotorConfig.Slot0.kV != logShooterKV.get() ||
            shooterMotorConfig.Slot0.kA != logShooterKA.get() ||
            shooterMotorConfig.Slot0.kP != logShooterKP.get() ||
            shooterMotorConfig.Slot0.kI != logShooterKI.get() ||
            shooterMotorConfig.Slot0.kD != logShooterKD.get()
        ) configureShooterMotors();
    }

    private void applyState() {
        switch (angleState) {
            case SHOOT:
                angleSetpoint = getShooterAngle();
                angleMotor.setControl(angleRequest.withPosition(angleSetpoint));
                break;
            case STORE:
                angleSetpoint = kHoodMinPosition;
                angleMotor.setControl(angleRequest.withPosition(angleSetpoint));
                break;
        }

        switch (shooterState) {
            case IDLE:
                shooterSetpoint = 0;
                shooterLeftMotor.setVoltage(0);
                break;
            case SHOOT:
                shooterSetpoint = getShooterVelocity();
                shooterLeftMotor.setControl(shootRequest.withVelocity(shooterSetpoint));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Shooter/Angle State", angleState);
        Logger.recordOutput("NewHopper/Shooter/Shooter State", shooterState);
        Logger.recordOutput("NewHopper/Shooter/Angle Setpoint", angleSetpoint);
        Logger.recordOutput("NewHopper/Shooter/Shooter Setpoint", shooterSetpoint);
        Logger.recordOutput("NewHopper/Shooter/Hood Manual Target", hoodTarget);
        Logger.recordOutput("NewHopper/Shooter/Shooter Manual Target", shooterTarget);

        if (logShooterDebug.get()) {
            Logger.recordOutput("NewHopper/Shooter/Angle Velocity", angleMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Angle Position", angleMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Angle Voltage", angleMotor.getMotorVoltage().getValueAsDouble());

            Logger.recordOutput("NewHopper/Shooter/Shooter Velocity", shooterLeftMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Shooter Position", shooterLeftMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Shooter Voltage", shooterLeftMotor.getMotorVoltage().getValueAsDouble());

            Logger.recordOutput("NewHopper/Shooter/Angle At Goal", isAngleWithinTolerance());
            Logger.recordOutput("NewHopper/Shooter/Shooter At Goal", isVelocityWithinTolerance());
        }
    }

    /** Manually adjusted flywheel velocity, in rotations per second. */
    public double getShooterVelocity() {
        return MathUtil.clamp(shooterTarget, kShooterMinVelocity, kShooterMaxVelocity);
    }

    /** Manually adjusted hood position, in motor revolutions. */
    public double getShooterAngle() {
        return MathUtil.clamp(hoodTarget, kHoodMinPosition, kHoodMaxPosition);
    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        return INSTANCE;
    }

    public void setAngleState(AngleState state) {
        angleState = state;
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public InstantCommand setAngleStateCommand(AngleState state) {
        return new InstantCommand(() -> setAngleState(state));
    }

    public InstantCommand setShooterStateCommand(ShooterState state) {
        return new InstantCommand(() -> setShooterState(state));
    }

    public AngleState getCurrentAngleState() {
        return angleState;
    }

    public ShooterState getCurrentShooterState() {
        return shooterState;
    }

    /* ========== MANUAL HOOD CONTROL ========== */

    /**
     * Moves the hood target by the given number of motor revolutions, clamped to the hood range.
     * Nudging the hood also takes it out of STORE so the adjustment is actually followed.
     */
    public void nudgeHoodTarget(double delta) {
        hoodTarget = MathUtil.clamp(hoodTarget + delta, kHoodMinPosition, kHoodMaxPosition);
        angleState = AngleState.SHOOT;
    }

    public void addManualHoodShift() {
        nudgeHoodTarget(kHoodStep);
    }

    public void subtractManualHoodShift() {
        nudgeHoodTarget(-kHoodStep);
    }

    public void resetManualHoodShift() {
        hoodTarget = kHoodMinPosition;
    }

    public double getHoodTarget() {
        return hoodTarget;
    }

    public InstantCommand nudgeHoodTargetCommand(double delta) {
        return new InstantCommand(() -> nudgeHoodTarget(delta));
    }

    /* ========== MANUAL FLYWHEEL CONTROL ========== */

    /** Moves the flywheel target by the given rotations per second, clamped to the flywheel range. */
    public void nudgeShooterTarget(double delta) {
        shooterTarget = MathUtil.clamp(shooterTarget + delta, kShooterMinVelocity, kShooterMaxVelocity);
    }

    public void addManualVelocityShift() {
        nudgeShooterTarget(kShooterStep);
    }

    public void subtractManualVelocityShift() {
        nudgeShooterTarget(-kShooterStep);
    }

    public void resetManualVelocityShift() {
        shooterTarget = kShooterDefaultVelocity;
    }

    public double getShooterTarget() {
        return shooterTarget;
    }

    public InstantCommand nudgeShooterTargetCommand(double delta) {
        return new InstantCommand(() -> nudgeShooterTarget(delta));
    }

    public InstantCommand resetShooterTargetCommand() {
        return new InstantCommand(() -> resetManualVelocityShift());
    }

    /* ========== HOOD ZEROING ========== */

    /**
     * Calls the hood's current position zero. The hood has no absolute encoder, so this has to be done
     * with the hood resting on its hard stop. Disabled only, so it can't be hit mid match.
     */
    public void zeroHoodEncoder() {
        if (!DriverStation.isDisabled()) return;
        angleMotor.setPosition(0);
        hoodTarget = kHoodMinPosition;
    }

    public Command zeroHoodEncoderCommand() {
        return new InstantCommand(() -> zeroHoodEncoder()).ignoringDisable(true);
    }

    /* ========== CHECKS ========== */

    public boolean isAngleWithinTolerance() {
        return isAngleWithinTolerance(2.75); // Motor revolutions // TODO: Retune for this robot's hood
    }

    public boolean isAngleWithinTolerance(double tolerance) {
        return Math.abs(angleSetpoint - angleMotor.getPosition().getValueAsDouble()) < tolerance;
    }

    public boolean isVelocityWithinTolerance() {
        return isVelocityWithinTolerance(16); // Rotations per second // TODO: Retune for this robot's flywheel
    }

    public boolean isVelocityWithinTolerance(double tolerance) {
        return Math.abs(shooterSetpoint - shooterLeftMotor.getVelocity().getValueAsDouble()) < tolerance;
    }

    public boolean isShooterReady() {
        return isAngleWithinTolerance() && isVelocityWithinTolerance();
    }
}
