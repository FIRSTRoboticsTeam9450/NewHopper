package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SidesweeperConstants;

public class SidesweeperSubsystem extends SubsystemBase {
    private static SidesweeperSubsystem INSTANCE;

    private TalonFX upperTransfer = new TalonFX(SidesweeperConstants.DeviceIDs.kUpperTransferMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX lowerTransfer = new TalonFX(SidesweeperConstants.DeviceIDs.kLowerTransferMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX rampMotor = new TalonFX(SidesweeperConstants.DeviceIDs.kRampMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public enum SidesweeperState {
    }

    public SidesweeperSubsystem() {
        configureSidesweeperMotors();
    }

    public void configureSidesweeperMotors() {
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

    public SidesweeperSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new SidesweeperSubsystem();
        return INSTANCE;
    }
}
