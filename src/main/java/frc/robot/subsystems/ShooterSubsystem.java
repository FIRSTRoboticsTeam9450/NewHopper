package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    public enum AngleState {}

    public enum ShooterState {}

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
