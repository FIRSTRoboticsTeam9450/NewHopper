package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TransferConstants;

public class HopperTransferSubsystem extends SubsystemBase {
    private static HopperTransferSubsystem INSTANCE;
    
    public enum TransferState {
        TRANSFER,
        IDLE
    }

    private TransferState currentTransferState = TransferState.IDLE;

    private TalonFX transferUpperMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferUpperMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX transferLowerMotor = new TalonFX(TransferConstants.DeviceIDs.kTransferLowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration transferMotorConfig = new TalonFXConfiguration();

    private VoltageOut transferRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    
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
    }

    private void publishLogs() {
    }

    public static HopperTransferSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new HopperTransferSubsystem();
        return INSTANCE;
    }    
}
