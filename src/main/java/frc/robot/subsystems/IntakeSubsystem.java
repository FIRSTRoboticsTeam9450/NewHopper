package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public static IntakeSubsystem instance;

    TalonFX deployMotor = new TalonFX(0, "nothing yet");

    TalonFX leftIntakeMotor = new TalonFX(0, "nothing yet");
    TalonFX rightIntakeMotor = new TalonFX(0, "nothing yet");
    
    TalonFXConfiguration config = new TalonFXConfiguration();

    private IntakeSubsystem() {
        configureIntakeMotors();
    }

    private void configureIntakeMotors() {
        config.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    }

    @Override
    public void periodic() {}

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }
}
