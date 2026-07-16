package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private static TurretSubsystem INSTANCE;

    public enum TurretState {
        AIM,
        IDLE
    }

    public enum SideState {
        TRANSFER,
        IDLE
    }

    private TurretState currentTurretState = TurretState.IDLE;
    private SideState currentSideState = SideState.IDLE;

    private double angleMax = 0; // Motor revolutions // TODO: Max turret angle
    private double angleMin = 0; // Motor revolutions // TODO: Min turret angle

    private TalonFX turretMotor = new TalonFX(TurretConstants.DeviceIDs.kTurretMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX sideMotor = new TalonFX(TurretConstants.DeviceIDs.kSideMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration sideMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logTurretAccel = new LoggedNetworkNumber("/Tuning/Turret/Turret/Accel", 0);
    private LoggedNetworkNumber logTurretVeloc = new LoggedNetworkNumber("/Tuning/Turret/Turret/Veloc",0);
    private LoggedNetworkNumber logTurretKS = new LoggedNetworkNumber("/Tuning/Turret/Turret/kS", 0);
    private LoggedNetworkNumber logTurretKV = new LoggedNetworkNumber("/Tuning/Turret/Turret/kV", 0);
    private LoggedNetworkNumber logTurretKA = new LoggedNetworkNumber("/Tuning/Turret/Turret/kA", 0);
    private LoggedNetworkNumber logTurretKP = new LoggedNetworkNumber("/Tuning/Turret/Turret/kP", 0);
    private LoggedNetworkNumber logTurretKI = new LoggedNetworkNumber("/Tuning/Turret/Turret/kI", 0);
    private LoggedNetworkNumber logTurretKD = new LoggedNetworkNumber("/Tuning/Turret/Turret/kD", 0);

    private MotionMagicVoltage turretRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut sideRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private TurretSubsystem() {
        configureTurretMotors();
        configureSideMotors();
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

        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = angleMax;
        turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
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

        turretMotor.getConfigurator().apply(turretMotorConfig);
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

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() {
    }
    
    private void publishLogs() {
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
}
