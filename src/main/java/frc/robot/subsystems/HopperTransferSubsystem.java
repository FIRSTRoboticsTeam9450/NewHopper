package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TransferConstants;

public class HopperTransferSubsystem extends SubsystemBase {
    private static HopperTransferSubsystem INSTANCE;
    
    public enum TransferState {
        TRANSFER,
        REVERSE,
        IDLE
    }

    private TransferState transferState = TransferState.IDLE;

    // Volts. The two rollers run at different speeds, which is why they are driven separately below
    // rather than as a follower pair
    private double speedVerticalTransfer = 12;
    private double speedBedTransfer = 4;
    private double speedTransferReverse = -2;
    private double speedTransferIdle = 0;

    // The upper transfer motor is the vertical roller, the lower one is the bed roller
    private TalonFX verticalRollerMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferUpperMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX bedRollerMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferLowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration transferMotorConfig = new TalonFXConfiguration();

    private VoltageOut verticalRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut bedRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logTransferDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Transfer", false);

    private double verticalSetpoint = 0;
    private double bedSetpoint = 0;

    private HopperTransferSubsystem() {
        configureTransferMotors();
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

        verticalRollerMotor.getConfigurator().apply(transferMotorConfig);
        bedRollerMotor.getConfigurator().apply(transferMotorConfig);

        // No Follower here on purpose. The bed roller runs slower than the vertical roller, so following
        // would force both to the same voltage
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
                verticalSetpoint = speedTransferIdle;
                bedSetpoint = speedTransferIdle;
                break;
            case TRANSFER:
                verticalSetpoint = speedVerticalTransfer;
                bedSetpoint = speedBedTransfer;
                break;
            case REVERSE:
                verticalSetpoint = speedTransferReverse;
                bedSetpoint = speedTransferReverse;
                break;
        }

        verticalRollerMotor.setControl(verticalRequest.withOutput(verticalSetpoint));
        bedRollerMotor.setControl(bedRequest.withOutput(bedSetpoint));
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Transfer/Transfer State", transferState);
        Logger.recordOutput("NewHopper/Transfer/Vertical Setpoint", verticalSetpoint);
        Logger.recordOutput("NewHopper/Transfer/Bed Setpoint", bedSetpoint);

        if (logTransferDebug.get()) {
            Logger.recordOutput("NewHopper/Transfer/Vertical Velocity", verticalRollerMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Vertical Voltage", verticalRollerMotor.getMotorVoltage().getValueAsDouble());

            Logger.recordOutput("NewHopper/Transfer/Bed Velocity", bedRollerMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Bed Voltage", bedRollerMotor.getMotorVoltage().getValueAsDouble());
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
