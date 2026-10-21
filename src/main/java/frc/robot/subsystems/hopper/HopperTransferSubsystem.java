package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import static frc.robot.RobotContainer.coordSub;

public class HopperTransferSubsystem extends SubsystemBase {
    
    public enum TransferState {
        TRANSFER,
        SLOW_TRANSFER,
        IDLE,
        EJECT
    }

    private double speedTransferTransfer = 10; // Volts // TODO: Transfer speed
    private double speedTransferSlow = 5;
    private double speedTransferIdle = 0; // Volts

    private TalonFX transferUpperMotor = new TalonFX(HopperTransferConstants.DeviceIDs.kTransferUpperMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX transferLowerMotor = new TalonFX(HopperTransferConstants.DeviceIDs.kTransferLowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private CANrange transferCanrange = new CANrange(HopperTransferConstants.DeviceIDs.kTransferCanrangeID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private LoggedNetworkNumber logFovCenterX = new LoggedNetworkNumber("/Tuning/Transfer/CANrange/FOV CX", 0);
    private LoggedNetworkNumber logFovCenterY = new LoggedNetworkNumber("/Tuning/Transfer/CANrange/FOV CY", 0);
    private LoggedNetworkNumber logFovRangeX = new LoggedNetworkNumber("/Tuning/Transfer/CANrange/FOV RX", 27);
    private LoggedNetworkNumber logFovRangeY = new LoggedNetworkNumber("/Tuning/Transfer/CANrange/FOV RY", 27);
    private LoggedNetworkNumber logFovDistance = new LoggedNetworkNumber("/Tuning/Transfer/CANrange/Trigger Inches", 16);

    private TalonFXConfiguration transferMotorConfig = new TalonFXConfiguration();

    private CANrangeConfiguration transferCanrangeConfig = new CANrangeConfiguration();

    private VoltageOut transferRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logTransferDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Transfer", false);

    private double transferSetpoint = 0;
    private double pauseForSideSweep = 0;

    private boolean slowHopper = false;
    
    public HopperTransferSubsystem() {
        configureTransferMotors();
        // transferUpperMotor.setControl(transferRequest.withOutput(4));
        configureCanrange();
    }

    private void configureTransferMotors() {
        transferMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        transferMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        transferMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        transferMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        transferMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        transferMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        transferMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        transferMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        transferUpperMotor.getConfigurator().apply(transferMotorConfig);
        transferLowerMotor.getConfigurator().apply(transferMotorConfig);

        transferLowerMotor.setControl(new Follower(transferUpperMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configureCanrange() {
        transferCanrangeConfig.ProximityParams.ProximityThreshold = Units.inchesToMeters(logFovDistance.get());

        transferCanrangeConfig.FovParams.FOVCenterX = logFovCenterX.get();
        transferCanrangeConfig.FovParams.FOVCenterY = logFovCenterY.get();
        transferCanrangeConfig.FovParams.FOVRangeX = logFovRangeX.get();
        transferCanrangeConfig.FovParams.FOVRangeY = logFovRangeY.get();

        transferCanrange.getConfigurator().apply(transferCanrangeConfig);
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
        switch (coordSub.getTransferState()) {
            case IDLE:
                transferSetpoint = speedTransferIdle;
                transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
                break;
            case TRANSFER:
                // if(Timer.getFPGATimestamp() - pauseForSideSweep > .25) {
                    transferSetpoint = speedTransferTransfer;
                    transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
                // }
                // else {
                //     transferSetpoint = 0;
                //     transferUpperMotor.setControl(transferRequest.withOutput(0));
                // }
                break;
            case SLOW_TRANSFER:
                transferSetpoint = speedTransferSlow;
                transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
                break;
            case EJECT:
                transferSetpoint = -4;
                transferUpperMotor.setControl(transferRequest.withOutput(transferSetpoint));
        }
    }

    private void publishLogs() {

        //TODO: Move
        Logger.recordOutput("NewHopper/Transfer/Transfer State", coordSub.getTransferState());
        Logger.recordOutput("NewHopper/Transfer/Transfer Setpoint", transferSetpoint);
        Logger.recordOutput("NewHopper/Transfer/Transfer Voltage", transferUpperMotor.getMotorVoltage().getValueAsDouble());

        if (logTransferDebug.get()) {
            Logger.recordOutput("NewHopper/Transfer/Transfer Velocity", transferUpperMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Transfer Position", transferUpperMotor.getPosition().getValueAsDouble());

            Logger.recordOutput("NewHopper/Transfer/Upper Supply Amps", transferUpperMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Upper Stator Amps", transferUpperMotor.getStatorCurrent().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Transfer/Lower Supply Amps", transferLowerMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("NewHopper/Transfer/Lower Stator Amps", transferLowerMotor.getStatorCurrent().getValueAsDouble());

            Logger.recordOutput("NewHopper/Transfer/Can See Fuel", canSeeFuel());
            Logger.recordOutput("NewHopper/Transfer/Fuel Distance Inches", getFuelDistance());
        }
    }

    public boolean canSeeFuel() {
        return transferCanrange.getIsDetected().getValue();
    }
    public double getFuelDistance() {
        return Units.metersToInches(transferCanrange.getDistance().getValueAsDouble());
    }

    public void updatedTransferState() {
        if(coordSub.getTransferState() == TransferState.TRANSFER) {
            pauseForSideSweep = Timer.getFPGATimestamp();
        }
    }

    public void setSlowHopper() {
        slowHopper = !slowHopper;
        if(slowHopper) {
            speedTransferTransfer = 2;
        }
        else {
            speedTransferTransfer = 5;
        }
    }

    public void updateTransferConstants() {
        if (
            transferCanrangeConfig.FovParams.FOVCenterX != logFovCenterX.get() ||
            transferCanrangeConfig.FovParams.FOVCenterY != logFovCenterY.get() ||
            transferCanrangeConfig.FovParams.FOVRangeX != logFovRangeX.get() ||
            transferCanrangeConfig.FovParams.FOVRangeY != logFovRangeY.get() ||
            transferCanrangeConfig.ProximityParams.ProximityThreshold != Units.inchesToMeters(logFovDistance.get())
        ) {
            configureCanrange();
        }
    }

}
