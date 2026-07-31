package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
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

    private DeployState deployState = DeployState.ZEROING;
    private IntakeState intakeState = IntakeState.IDLE;

    private double angleDeployStore = 0; // Motor revolutions // TODO: Store angle
    // TODO: VERIFY SIGN. This measured value says deploying out is NEGATIVE revolutions, but
    // configureDeployMotors() below claims "Positive = Intake out". One of the two is wrong.
    // Use /Tuning/Intake/Deploy/Test Position to find the real direction before trusting either.
    private double angleDeployIntake = -17.798; // Motor revolutions // TODO: Intake angle

    private double speedIntakeIntake = 0; // Volts // TODO: Intake speed
    private double speedIntakeIdle = 0; // Volts

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.DRIVEBASE_CANBUS);
    private TalonFX intakeLeftMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, RobotConstants.RIO_BUS);
    private TalonFX intakeRightMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeRightMotorID, RobotConstants.RIO_BUS);
    
    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    // Accel and Veloc MUST stay non-zero. For non-Expo Motion Magic, 0 does NOT mean "unlimited"
    // (that only applies to Motion Magic Expo) - it means the generated profile can never advance,
    // so the motor holds its current position no matter what position you command.
    // These are deliberately slow starting values; tune upward once direction is confirmed.
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 20);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc", 10);
    private LoggedNetworkNumber logDeployKS = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kS", 0);
    private LoggedNetworkNumber logDeployKV = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kV", 0);
    private LoggedNetworkNumber logDeployKA = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kA", 0);
    private LoggedNetworkNumber logDeployKP = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kP", 5);
    private LoggedNetworkNumber logDeployKI = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kI", 0);
    private LoggedNetworkNumber logDeployKD = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kD", 5);
    private LoggedNetworkNumber logDeployKG = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kG", 0);

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private Follower intakeFollowRequest = new Follower(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, MotorAlignmentValue.Opposed);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", true);

    /** Target used by the DRIVER.x() test keybind, so deploy travel/direction can be probed without a redeploy. */
    private LoggedNetworkNumber logDeployTestPosition = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Test Position", 17.6);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;

    private IntakeSubsystem() {
        configureDeployMotors();
        configureIntakeMotors();

        // The deploy motor may not be enumerated on the CAN bus yet this early in startup. If the
        // zeroing silently fails, every position command afterwards is referenced to garbage, so say so.
        StatusCode zeroStatus = deployMotor.setPosition(0);
        if (!zeroStatus.isOK()) {
            DriverStation.reportWarning(
                "Intake deploy motor failed to zero at startup: " + zeroStatus.toString(), false);
        }
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

        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        intakeLeftMotor.getConfigurator().apply(intakeMotorConfig);
        intakeRightMotor.getConfigurator().apply(intakeMotorConfig);

        // Command the wheels off immediately. A Talon keeps the last control request it was sent, so
        // without this the rollers pick straight back up at the last commanded voltage the moment the
        // robot is enabled - even across a code restart.
        applyIntakeWheels();
    }

    /**
     * Pushes the current wheel setpoint to the motors. Called every loop so the rollers only ever
     * run when something has actually asked them to, and so a device reboot cannot leave the right
     * motor without its follower assignment.
     */
    private void applyIntakeWheels() {
        intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
        intakeRightMotor.setControl(intakeFollowRequest);
    }

    @Override
    public void periodic() {
        // Left disabled on purpose: the STORE case re-commands position 0 every loop, which would
        // immediately overwrite anything the direct setPosition()/runInOut() keybinds ask for.
        // Do not re-enable this while the manual test keybinds in RobotContainer are active.
        //applyState();
        applyIntakeWheels();
        publishLogs();
    }

    private void applyState() {
        switch (deployState) {
            case ZEROING:
                deployMotor.setPosition(0);
                deployState = DeployState.STORE;
                break;
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
            case BRAKE:
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("NewHopper/Intake/Angle State", deployState);
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

    public void setDeployState(DeployState state) {
        deployState = state;
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public void setPosition(double pos) {
        deploySetpoint = pos;
        deployMotor.setControl(deployRequest.withPosition(pos));
    }

    /** Target for the DRIVER.x() test keybind, live-editable at /Tuning/Intake/Deploy/Test Position. */
    public double getTestPosition() {
        return logDeployTestPosition.get();
    }

    /** Holds the arm wherever it currently is, rather than driving it back to the zero reference. */
    public void holdPosition() {
        setPosition(deployMotor.getPosition().getValueAsDouble());
    }

    /** Drives the arm back to the stored (zero) reference. */
    public void storePosition() {
        setPosition(1);
    }

    public void runInOut(boolean out) {
        intakeSetpoint = out ? -12 : 12;
        applyIntakeWheels();
    }

    public void killIntakeWheels() {
        intakeSetpoint = 0;
        applyIntakeWheels();
    }

    public InstantCommand setAngleStateCommand(DeployState state) {
        return new InstantCommand(() -> setDeployState(state));
    }

    public InstantCommand setIntakeStateCommand(IntakeState state) {
        return new InstantCommand(() -> setIntakeState(state));
    }
}
