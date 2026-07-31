package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private static TurretSubsystem INSTANCE;

    public enum SideState {
        TRANSFER,
        IDLE
    }

    public enum TurretState {
        /** Do nothing, sit there and look pretty */
        DISABLED,
        /** Goes to store, then once there, goes to aiming */
        INIT,
        /** Holding current position, no active tracking */
        AIMING,
        /** Moving to safe storage position (0°) */
        STORING,

        /**  If hood flips off */
        EMERGENCY
    }

    private SideState sideState = SideState.IDLE;
    private TurretState turretState = TurretState.DISABLED;

    private double angleMax = 0; // Motor revolutions // TODO: Max turret angle
    private double angleMin = 0; // Motor revolutions // TODO: Min turret angle

    private double speedSideTransfer = 0; // Volts // TODO: Transfer speed
    private double speedSideIdle = 0; // Volts

    private CANcoder turretEncoder = new CANcoder(TurretConstants.DeviceIDs.kTurretEncoderID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFX sideMotor = new TalonFX(TurretConstants.DeviceIDs.kSideMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX turretMotor = new TalonFX(TurretConstants.DeviceIDs.kTurretMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

    private TalonFXConfiguration sideMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    // Accel and Veloc MUST stay non-zero. For non-Expo Motion Magic, 0 does NOT mean "unlimited"
    // (that only applies to Motion Magic Expo) - it means the generated profile can never advance,
    // so the motor holds its current position no matter what you command.
    private LoggedNetworkNumber logTurretAccel = new LoggedNetworkNumber("/Tuning/Turret/Turret/Accel", 20);
    private LoggedNetworkNumber logTurretVeloc = new LoggedNetworkNumber("/Tuning/Turret/Turret/Veloc", 10);
    private LoggedNetworkNumber logTurretKS = new LoggedNetworkNumber("/Tuning/Turret/Turret/kS", 0);
    private LoggedNetworkNumber logTurretKV = new LoggedNetworkNumber("/Tuning/Turret/Turret/kV", 0);
    private LoggedNetworkNumber logTurretKA = new LoggedNetworkNumber("/Tuning/Turret/Turret/kA", 0);
    private LoggedNetworkNumber logTurretKP = new LoggedNetworkNumber("/Tuning/Turret/Turret/kP", 0);
    private LoggedNetworkNumber logTurretKI = new LoggedNetworkNumber("/Tuning/Turret/Turret/kI", 0);
    private LoggedNetworkNumber logTurretKD = new LoggedNetworkNumber("/Tuning/Turret/Turret/kD", 0);

    private VoltageOut sideRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private MotionMagicVoltage turretRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logTurretDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Turret", false);

    private double sideSetpoint = 0;
    private double turretSetpoint = 0;

    private TurretSubsystem() {
        configureTurretEncoders();

        configureSideMotors();
        configureTurretMotors();
    }

    private void configureTurretEncoders() {
        turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        turretEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Positive = Clockwise, Negative = Counterclockwise

        turretEncoderConfig.MagnetSensor.MagnetOffset = TurretConstants.kTurretEncoderMagnetOffset;

        turretEncoder.getConfigurator().apply(turretEncoderConfig);
    }

    private void configureSideMotors() {
        sideMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        sideMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        sideMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        sideMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        sideMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        sideMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        sideMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        sideMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        sideMotor.getConfigurator().apply(sideMotorConfig);
    }

    private void configureTurretMotors() {
        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Clockwise, Negative = Counterclockwise

        turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
        turretMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        turretMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 15;

        turretMotorConfig.Voltage.PeakForwardVoltage = 7;
        turretMotorConfig.Voltage.PeakReverseVoltage = -7;

        turretMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turretMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // TODO: re-enable once angleMax/angleMin are real measured values. Enabling them against
        // the placeholder 0/0 pins the turret to exactly 0 - it cannot rotate in either direction.
        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = angleMax;
        turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = angleMin;

        turretMotorConfig.MotionMagic.MotionMagicAcceleration = logTurretAccel.get();
        turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = logTurretVeloc.get();

        turretMotorConfig.Slot0.kS = logTurretKS.get();
        turretMotorConfig.Slot0.kV = logTurretKV.get();
        turretMotorConfig.Slot0.kA = logTurretKA.get();
        turretMotorConfig.Slot0.kP = logTurretKP.get();
        turretMotorConfig.Slot0.kI = logTurretKI.get();
        turretMotorConfig.Slot0.kD = logTurretKD.get();
        // No kG or GravityType because it spins horizontally and does not have gravity

        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turretMotorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        turretMotorConfig.Feedback.RotorToSensorRatio = TurretConstants.kTurretRotorToSensorRatio;
        turretMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.kTurretSensorToMechanismRatio;

        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() { // TODO: applyState()
        switch (sideState) {
            case IDLE:
                sideSetpoint = speedSideIdle;
                sideMotor.setControl(sideRequest.withOutput(sideSetpoint));
                break;
            case TRANSFER:
                sideSetpoint = speedSideTransfer;
                sideMotor.setControl(sideRequest.withOutput(sideSetpoint));
                break;
        }

        switch (turretState) {
            case DISABLED:
                turretMotor.stopMotor();
                break;
            case INIT:
                if (isAtProfiledGoal(20) && ShooterSubsystem.getInstance().getCurrentAngleState() != ShooterSubsystem.AngleState.STORE) { //zeroing
                        turretState = TurretState.AIMING;
                    }
                break;
            case AIMING:
                break;
            case EMERGENCY:
                break;
            case STORING:
                break;
            
        }
    }
    
    private void publishLogs() {
        Logger.recordOutput("NewHopper/Turret/Turret Relative Position", turretEncoder.getPosition().getValueAsDouble());
        Logger.recordOutput("NewHopper/Turret/Turret Absolute Position", turretEncoder.getAbsolutePosition().getValueAsDouble());

        Logger.recordOutput("NewHopper/Turret/Side State", sideState);
        Logger.recordOutput("NewHopper/Turret/Turret State", turretState);
        Logger.recordOutput("NewHopper/Turret/Side Setpoint", sideSetpoint);
        Logger.recordOutput("NewHopper/Turret/Turret Setpoint", turretSetpoint);

        if (logTurretDebug.get()) {
            Logger.recordOutput("NewHopper/Turret/Turret Encoder Velocity", turretEncoder.getVelocity().getValueAsDouble());

            Logger.recordOutput("NewHopper/Turret/Side Velocity", sideMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Turret/Side Position", sideMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Turret/Side Voltage", sideMotor.getMotorVoltage().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Turret/Turret Velocity", turretMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Turret/Turret Position", turretMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Turret/Turret Voltage", turretMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    public void updateConstants() {
        if (
            turretMotorConfig.MotionMagic.MotionMagicAcceleration != logTurretAccel.get() ||
            turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logTurretVeloc.get() ||
            turretMotorConfig.Slot0.kS != logTurretKS.get() ||
            turretMotorConfig.Slot0.kV != logTurretKV.get() ||
            turretMotorConfig.Slot0.kA != logTurretKA.get() ||
            turretMotorConfig.Slot0.kP != logTurretKP.get() ||
            turretMotorConfig.Slot0.kI != logTurretKI.get() ||
            turretMotorConfig.Slot0.kD != logTurretKD.get()
        ) configureTurretMotors();
    }

    public static TurretSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new TurretSubsystem();
        return INSTANCE;
    }
    
    public void setSideState(SideState state) {
        sideState = state;
    }

    public void setTurretState(TurretState state) {
        turretState = state;
    }

    public boolean isAtProfiledGoal(double tolerance) {
        return false;
    }

    public InstantCommand setSideStateCommand(SideState state) {
        return new InstantCommand(() -> setSideState(state));
    }

    public InstantCommand setTurretStateCommand(TurretState state) {
        return new InstantCommand(() -> setTurretState(state));
    }
}
