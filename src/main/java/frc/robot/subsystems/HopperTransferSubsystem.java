package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperTransferSubsystem extends SubsystemBase {

    public static HopperTransferSubsystem instance;

    TalonFX leadTransferMotor;
    TalonFX followerTransferMotor;

    TalonFXConfiguration config = new TalonFXConfiguration();

    public enum TransferState {

    }

    //private MotionMagic
    
    private HopperTransferSubsystem() {}

    public void configureTransferMotors() {}

    public boolean isJammed() {
        return false;
    }

    @Override
    public void periodic() {}

    public static HopperTransferSubsystem getInstance() {
        if (instance == null) {
            instance = new HopperTransferSubsystem();
        }
        return instance;
    }    
}
