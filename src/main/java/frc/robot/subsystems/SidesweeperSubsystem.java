package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SidesweeperSubsystem extends SubsystemBase {

    TalonFX upperTransfer = new TalonFX(0, "nothing yet");
    TalonFX lowerTransfer = new TalonFX(0, "nothing yet");
    TalonFX rampMotor = new TalonFX(0, "nothing yet");

    TalonFXConfiguration config = new TalonFXConfiguration();

    public SidesweeperSubsystem() {}

    public void configureSidesweeperMotors() {

    }
}
