package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TransferConstants;

public class HopperTransferSubsystem extends SubsystemBase {
    private static HopperTransferSubsystem INSTANCE;
    
    public enum TransferState {
        TRANSFER,
        IDLE
    }

    private TransferState transferState = TransferState.IDLE;

    private double speedTransferTransfer = 0; // Volts // TODO: Transfer speed
    private double speedTransferIdle = 0; // Volts

    private TalonFX transferUpperMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferUpperMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX transferLowerMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferLowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration transferMotorConfig = new TalonFXConfiguration();

    private VoltageOut transferRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logTransferDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Transfer", false);

    private double transferSetpoint = 0;
    
    private HopperTransferSubsystem() {
        configureTransferMotors();
        transferUpperMotor.setControl(transferRequest.withOutput(4));
    }

    private void configureTransferMotors() {
        transferMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        transferMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        transferMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        transferMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        transferMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        transferMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        transferMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        transferMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        transferUpperMotor.getConfigurator().apply(transferMotorConfig);
        transferLowerMotor.getConfigurator().apply(transferMotorConfig);

        transferLowerMotor.setControl(new Follower(transferUpperMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public boolean isJammed() {
        return false;
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (transferState) {
            case IDLE:
                transferSetpoint = speedTransferIdle;
                transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
                break;
            case TRANSFER:
                transferSetpoint = speedTransferTransfer;
                transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Transfer/Transfer State", transferState);
        Logger.recordOutput("NewHopper/Transfer/Transfer Setpoint", transferSetpoint);

        if (logTransferDebug.get()) {
            Logger.recordOutput("NewHopper/Transfer/Transfer Velocity", transferUpperMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Transfer Position", transferUpperMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Transfer Voltage", transferUpperMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    public static HopperTransferSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new HopperTransferSubsystem();
        return INSTANCE;
    }

    public void setTransferState(TransferState state) {
        transferState = state;
    }

    public InstantCommand setTransferStateCommand(TransferState state) {
        return new InstantCommand(() -> setTransferState(state));
    }
}
