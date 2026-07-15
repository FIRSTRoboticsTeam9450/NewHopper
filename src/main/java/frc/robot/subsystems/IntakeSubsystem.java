package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    
    public enum AngleState {
    }

    public enum IntakeState {
    }

    private AngleState currentAngleState;
    private IntakeState currentIntakeState;

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX leftIntakeMotor = new TalonFX(IntakeConstants.DeviceIDs.kLeftIntakeMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX rightIntakeMotor = new TalonFX(IntakeConstants.DeviceIDs.kRightIntakeMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private IntakeSubsystem() {
        configureIntakeMotors();
    }

    private void configureIntakeMotors() {
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakeSubsystem();
        return INSTANCE;
    }
}
