package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    
    public enum DeployState {
        ZEROING,
        DEPLOY,
        STORE
    }

    public enum IntakeState {
        INTAKE,
        BRAKE,
        IDLE
    }

    private DeployState angleState = DeployState.STORE;
    private IntakeState intakeState = IntakeState.IDLE;

    private double angleDeployStore = 0; // Motor revolutions // TODO: Store angle
    private double angleDeployIntake = 0; // Motor revolutions // TODO: Intake angle

    private double speedIntakeIntake = 0; // Volts // TODO: Intake speed
    private double speedIntakeIdle = 0; // Volts

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX intakeLeftMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX intakeRightMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeRightMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 0);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc",0);
    private LoggedNetworkNumber logDeployKS = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kS", 0);
    private LoggedNetworkNumber logDeployKV = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kV", 0);
    private LoggedNetworkNumber logDeployKA = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kA", 0);
    private LoggedNetworkNumber logDeployKP = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kP", 0);
    private LoggedNetworkNumber logDeployKI = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kI", 0);
    private LoggedNetworkNumber logDeployKD = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kD", 0);
    private LoggedNetworkNumber logDeployKG = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kG", 0);

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", false);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;

    private IntakeSubsystem() {
        configureDeployMotors();
        configureIntakeMotors();
    }

    private void configureDeployMotors() {
        deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Intake out, Negative = Intake in

        deployMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        deployMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        deployMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        deployMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        deployMotorConfig.MotionMagic.MotionMagicAcceleration = logDeployAccel.get();
        deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = logDeployVeloc.get();

        deployMotorConfig.Slot0.kS = logDeployKS.get();
        deployMotorConfig.Slot0.kV = logDeployKV.get();
        deployMotorConfig.Slot0.kA = logDeployKA.get();
        deployMotorConfig.Slot0.kP = logDeployKP.get();
        deployMotorConfig.Slot0.kI = logDeployKI.get();
        deployMotorConfig.Slot0.kD = logDeployKD.get();
        deployMotorConfig.Slot0.kG = logDeployKG.get();
        deployMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        deployMotor.getConfigurator().apply(deployMotorConfig);
    }

    private void configureIntakeMotors() {
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        intakeLeftMotor.getConfigurator().apply(intakeMotorConfig);
        intakeRightMotor.getConfigurator().apply(intakeMotorConfig);

        intakeRightMotor.setControl(new Follower(intakeLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (angleState) {
            case DEPLOY:
                deploySetpoint = angleDeployIntake;
                deployMotor.setControl(deployRequest.withPosition(deploySetpoint));
                break;
            case STORE:
                deploySetpoint = angleDeployStore;
                deployMotor.setControl(deployRequest.withPosition(deploySetpoint));
                break;
        }

        switch (intakeState) {
            case IDLE:
                intakeSetpoint = speedIntakeIdle;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
                break;
            case INTAKE:
                intakeSetpoint = speedIntakeIntake;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Intake/Angle State", angleState);
        Logger.recordOutput("NewHopper/Intake/Intake State", intakeState);
        Logger.recordOutput("NewHopper/Intake/Deploy Setpoint", deploySetpoint);
        Logger.recordOutput("NewHopper/Intake/Intake Setpoint", intakeSetpoint);

        if (logIntakeDebug.get()) {
            Logger.recordOutput("NewHopper/Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Position", deployMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Intake/Intake Velocity", intakeLeftMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Intake Position", intakeLeftMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Intake Voltage", intakeLeftMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    public void updateConstants() {
        if (
            deployMotorConfig.MotionMagic.MotionMagicAcceleration != logDeployAccel.get() ||
            deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logDeployVeloc.get() ||
            deployMotorConfig.Slot0.kS != logDeployKS.get() ||
            deployMotorConfig.Slot0.kV != logDeployKV.get() ||
            deployMotorConfig.Slot0.kA != logDeployKA.get() ||
            deployMotorConfig.Slot0.kP != logDeployKP.get() ||
            deployMotorConfig.Slot0.kI != logDeployKI.get() ||
            deployMotorConfig.Slot0.kD != logDeployKD.get() ||
            deployMotorConfig.Slot0.kG != logDeployKG.get()
        ) configureDeployMotors();
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakeSubsystem();
        return INSTANCE;
    }

    public void setAngleState(DeployState state) {
        angleState = state;
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public InstantCommand setAngleStateCommand(DeployState state) {
        return new InstantCommand(() -> setAngleState(state));
    }

    public InstantCommand setIntakeStateCommand(IntakeState state) {
        return new InstantCommand(() -> setIntakeState(state));
    }
}
