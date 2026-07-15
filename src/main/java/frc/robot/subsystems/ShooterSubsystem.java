package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

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

    TalonFX angleMotor;
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;

    public ShooterSubsystem() {}

    private void configureAngleMotors() {}

    private void configureShooterMotors() {}

    public void addData() {}

    @Override
    public void periodic() {}

    public void applyState() {}

    public double getShooterVelocity() {
        return 0;
    }

    public double getShooterAngle() {
        return 0;
    }
}
