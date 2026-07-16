package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem INSTANCE;

    public enum AngleState {
        SHOOT,
        STORE
    }

    public enum ShooterState {
        SHOOT,
        IDLE
    }

    private AngleState angleState = AngleState.STORE;
    private ShooterState shooterState = ShooterState.IDLE;

    private double angleMax = 0; // Motor revolutions // TODO: Max shooter angle

    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterLeftMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterRightMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    
    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logAngleAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Accel", 0);
    private LoggedNetworkNumber logAngleVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Veloc",0);
    private LoggedNetworkNumber logAngleKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", 0);
    private LoggedNetworkNumber logAngleKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0);
    private LoggedNetworkNumber logAngleKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0);
    private LoggedNetworkNumber logAngleKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 0);
    private LoggedNetworkNumber logAngleKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
    private LoggedNetworkNumber logAngleKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
    private LoggedNetworkNumber logAngleKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0);
    private LoggedNetworkNumber logShooterAccel = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/Accel", 0);
    private LoggedNetworkNumber logShooterKS = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kS", 0);
    private LoggedNetworkNumber logShooterKV = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kV", 0);
    private LoggedNetworkNumber logShooterKA = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kA", 0);
    private LoggedNetworkNumber logShooterKP = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kP", 0);
    private LoggedNetworkNumber logShooterKI = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kI", 0);
    private LoggedNetworkNumber logShooterKD = new LoggedNetworkNumber("/Tuning/Shooter/Shooter/kD", 0);

    private MotionMagicVoltage angleRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private MotionMagicVelocityVoltage shootRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logShooterDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    private double angleSetpoint = 0;
    private double shooterSetpoint = 0;

    private ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();
    }

    private void configureAngleMotors() {
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Up, Negative = Down

        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = angleMax;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        angleMotorConfig.MotionMagic.MotionMagicAcceleration = logAngleAccel.get();
        angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = logAngleVeloc.get();

        angleMotorConfig.Slot0.kS = logAngleKS.get();
        angleMotorConfig.Slot0.kV = logAngleKV.get();
        angleMotorConfig.Slot0.kA = logAngleKA.get();
        angleMotorConfig.Slot0.kP = logAngleKP.get();
        angleMotorConfig.Slot0.kI = logAngleKI.get();
        angleMotorConfig.Slot0.kD = logAngleKD.get();
        angleMotorConfig.Slot0.kG = logAngleKG.get();
        angleMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        angleMotor.getConfigurator().apply(angleMotorConfig);
    }

    private void configureShooterMotors() {
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive = Outtake, Negative = Intake

        shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;

        shooterMotorConfig.MotionMagic.MotionMagicAcceleration = logShooterAccel.get();
        // No CruiseVelocity because it's set to run to a specific velocity

        shooterMotorConfig.Slot0.kS = logShooterKS.get();
        shooterMotorConfig.Slot0.kV = logShooterKV.get();
        shooterMotorConfig.Slot0.kA = logShooterKA.get();
        shooterMotorConfig.Slot0.kP = logShooterKP.get();
        shooterMotorConfig.Slot0.kI = logShooterKI.get();
        shooterMotorConfig.Slot0.kD = logShooterKD.get();
        // No kG or GravityType because it spins freely and does not have gravity

        shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
        shooterRightMotor.getConfigurator().apply(shooterMotorConfig);

        shooterRightMotor.setControl(new Follower(shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void addData() {
    }

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    public void updateConstants() {
        if (
            angleMotorConfig.MotionMagic.MotionMagicAcceleration != logAngleAccel.get() ||
            angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logAngleVeloc.get() ||
            angleMotorConfig.Slot0.kS != logAngleKS.get() ||
            angleMotorConfig.Slot0.kV != logAngleKV.get() ||
            angleMotorConfig.Slot0.kA != logAngleKA.get() ||
            angleMotorConfig.Slot0.kP != logAngleKP.get() ||
            angleMotorConfig.Slot0.kI != logAngleKI.get() ||
            angleMotorConfig.Slot0.kD != logAngleKD.get() ||
            angleMotorConfig.Slot0.kG != logAngleKG.get()
        ) configureAngleMotors();
        
        if (
            shooterMotorConfig.MotionMagic.MotionMagicAcceleration != logShooterAccel.get() ||
            shooterMotorConfig.Slot0.kS != logShooterKS.get() ||
            shooterMotorConfig.Slot0.kV != logShooterKV.get() ||
            shooterMotorConfig.Slot0.kA != logShooterKA.get() ||
            shooterMotorConfig.Slot0.kP != logShooterKP.get() ||
            shooterMotorConfig.Slot0.kI != logShooterKI.get() ||
            shooterMotorConfig.Slot0.kD != logShooterKD.get()
        ) configureShooterMotors();
    }

    private void applyState() { // TODO: applyState()
        switch (angleState) {
            case SHOOT:
                break;
            case STORE:
                break;
        }

        switch (shooterState) {
            case IDLE:
                break;
            case SHOOT:
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Shooter/Angle State", angleState);
        Logger.recordOutput("NewHopper/Shooter/Shooter State", shooterState);
        Logger.recordOutput("NewHopper/Shooter/Angle Setpoint", angleSetpoint);
        Logger.recordOutput("NewHopper/Shooter/Shooter Setpoint", shooterSetpoint);

        if (logShooterDebug.get()) {
            Logger.recordOutput("NewHopper/Shooter/Angle Velocity", angleMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Angle Position", angleMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Angle Voltage", angleMotor.getMotorVoltage().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Shooter/Shooter Velocity", shooterLeftMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Shooter Position", shooterLeftMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Shooter Voltage", shooterLeftMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    public double getShooterVelocity() {
        return 0;
    }

    public double getShooterAngle() {
        return 0;
    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        return INSTANCE;
    }
    
    public void setAngleState(AngleState state) {
        angleState = state;
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public InstantCommand setAngleStateCommand(AngleState state) {
        return new InstantCommand(() -> setAngleState(state));
    }

    public InstantCommand setShooterStateCommand(ShooterState state) {
        return new InstantCommand(() -> setShooterState(state));
    }
}
