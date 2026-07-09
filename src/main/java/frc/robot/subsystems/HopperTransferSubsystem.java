package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperTransferSubsystem extends SubsystemBase {

    TalonFX leadTransferMotor;
    TalonFX followerTransferMotor;

    TalonFXConfiguration config = new TalonFXConfiguration();

    //private MotionMagic
    
    private HopperTransferSubsystem() {}

    public void configureTransferMotors() {}
}
