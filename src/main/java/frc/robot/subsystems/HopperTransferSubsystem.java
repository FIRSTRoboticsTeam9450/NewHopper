package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TransferConstants;

public class HopperTransferSubsystem extends SubsystemBase {
    private static HopperTransferSubsystem INSTANCE;

    private TalonFX leadTransferMotor = new TalonFX(TransferConstants.DeviceIDs.kLeadTransferMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX followerTransferMotor = new TalonFX(TransferConstants.DeviceIDs.kFollowerTransferMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public enum TransferState {
    }
    
    private HopperTransferSubsystem() {
        configureTransferMotors();
    }

    public void configureTransferMotors() {
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

    public void updateConstants() {
    }

    public static HopperTransferSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new HopperTransferSubsystem();
        return INSTANCE;
    }    
}
