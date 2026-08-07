package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SidesweeperConstants;

public class SidesweeperSubsystem extends SubsystemBase {
    private static SidesweeperSubsystem INSTANCE;

    public enum RampState {
        TRANSFER,
        IDLE
    }

    public enum SidesweeperState {
        TRANSFER,
        IDLE
    }

    private RampState rampState = RampState.IDLE;
    private SidesweeperState sidesweeperState = SidesweeperState.IDLE;

    private double speedRampTransfer = 10; // Volts // TODO: Transfer speed
    private double speedRampIdle = 0; // Volts
    
    private double speedSidesweeperTransfer = 10; // Volts // TODO: Transfer speed
    private double speedSidesweeperIdle = 0; // Volts

    private TalonFX rampMotor = new TalonFX(SidesweeperConstants.DeviceIDs.kRampMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX sidesweeperUpperMotor = new TalonFX(SidesweeperConstants.DeviceIDs.kSidesweeperUpperMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX sidesweeperLowerMotor = new TalonFX(SidesweeperConstants.DeviceIDs.kSidesweeperLowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    private TalonFX verticalRollerMotor = new TalonFX(SidesweeperConstants.DeviceIDs.kVerticalRollerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration sidesweeperMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rampMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration verticalRollerMotorConfig = new TalonFXConfiguration();

    private VoltageOut rampRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut sidesweepRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut verticalRollerRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logSidesweeperDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Sidesweeper", false);

    private double rampSetpoint = 0;
    private double verticalRollerSetpoint = 0;
    private double sidesweeperSetpoint = 0;

    public SidesweeperSubsystem() {
        configureRampMotors();
        configureSidesweeperMotors();
        configureVerticalRollerMotor();
        // sidesweeperUpperMotor.setControl(sidesweepRequest.withOutput(10));
        // rampMotor.setControl(rampRequest.withOutput(12));
    }

    private void configureVerticalRollerMotor() {
        verticalRollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        verticalRollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        verticalRollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        verticalRollerMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        verticalRollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        verticalRollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        verticalRollerMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        verticalRollerMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        verticalRollerMotor.getConfigurator().apply(verticalRollerMotorConfig);
    }

    private void configureRampMotors() {
        rampMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rampMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Intake, Negative = Outtake

        rampMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rampMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        rampMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rampMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        rampMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        rampMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        rampMotor.getConfigurator().apply(rampMotorConfig);
    }

    private void configureSidesweeperMotors() {
        sidesweeperMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        sidesweeperMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Intake, Negative = Outtake

        sidesweeperMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        sidesweeperMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        sidesweeperMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        sidesweeperMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        sidesweeperMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        sidesweeperMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;

        sidesweeperUpperMotor.getConfigurator().apply(sidesweeperMotorConfig);
        sidesweeperLowerMotor.getConfigurator().apply(sidesweeperMotorConfig);

        sidesweeperLowerMotor.setControl(new Follower(sidesweeperUpperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (rampState) {
            case IDLE:
                rampSetpoint = speedRampIdle;
                verticalRollerSetpoint = 0;
                verticalRollerMotor.setControl(verticalRollerRequest.withOutput(verticalRollerSetpoint));
                rampMotor.setControl(rampRequest.withOutput(rampSetpoint));
                break;
            case TRANSFER:
                rampSetpoint = speedRampTransfer;
                verticalRollerSetpoint = 12;
                verticalRollerMotor.setControl(verticalRollerRequest.withOutput(verticalRollerSetpoint));
                rampMotor.setControl(rampRequest.withOutput(rampSetpoint));
                break;
        }

        switch (sidesweeperState) {
            case IDLE:
                sidesweeperSetpoint = speedSidesweeperIdle;
                sidesweeperUpperMotor.setControl(sidesweepRequest.withOutput(sidesweeperSetpoint));
                break;
            case TRANSFER:
                sidesweeperSetpoint = speedSidesweeperTransfer;
                sidesweeperUpperMotor.setControl(sidesweepRequest.withOutput(sidesweeperSetpoint));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Sidesweeper/Ramp State", rampState);
        Logger.recordOutput("NewHopper/Sidesweeper/Sidesweeper State", sidesweeperState);
        Logger.recordOutput("NewHopper/Sidesweeper/Ramp Setpoint", rampSetpoint);
        Logger.recordOutput("NewHopper/Sidesweeper/Sidesweeper Setpoint", sidesweeperSetpoint);

        if (logSidesweeperDebug.get()) {
            Logger.recordOutput("NewHopper/Sidesweeper/Ramp Velocity", rampMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Sidesweeper/Ramp Position", rampMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Sidesweeper/Ramp Voltage", rampMotor.getMotorVoltage().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Sidesweeper/Sidesweeper Velocity", sidesweeperUpperMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Sidesweeper/Sidesweeper Position", sidesweeperUpperMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Sidesweeper/Sidesweeper Voltage", sidesweeperUpperMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    public static SidesweeperSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new SidesweeperSubsystem();
        return INSTANCE;
    }
    
    public void setRampState(RampState state) {
        rampState = state;
    }

    public void setSidesweeperState(SidesweeperState state) {
        sidesweeperState = state;
    }

    public InstantCommand setRampStateCommand(RampState state) {
        return new InstantCommand(() -> setRampState(state));
    }

    public InstantCommand setSidesweeperStateCommand(SidesweeperState state) {
        return new InstantCommand(() -> setSidesweeperState(state));
    }
}
