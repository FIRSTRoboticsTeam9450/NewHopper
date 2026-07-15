package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem INSTANCE;

    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX leftShooterMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX rightShooterMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public enum AngleState {
        DISABLED,
        ZEROING,
        AIMING,
        DEADZONE,
        STORING,
        TUNING
    }

    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING,
        HOLD_IN
    }

    private ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();
    }

    private void configureAngleMotors() {
    }

    private void configureShooterMotors() {
    }

    public void addData() {
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    public void updateConstants() {
    }

    private void applyState() {
    }

    private void publishLogs() {
    }

    public double getShooterVelocity() {
        return 0;
    }

    public double getShooterAngle() {
        return 0;
    }

    public ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        return INSTANCE;
    }
}
