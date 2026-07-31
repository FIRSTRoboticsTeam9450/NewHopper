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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private double angleDeployIntake = -17.798; // Motor revolutions // TODO: Intake angle

    /* ========== DEPLOY TARGET ========== */
    public static final double kDeployMinPosition = 0; // Motor revolutions, intake fully stored
    public static final double kDeployMaxPosition = 8; // Motor revolutions, intake fully deployed
    public static final double kDeployStep = 0.5; // Motor revolutions moved per button press
    private static final double kDeployStallTolerance = 0.01; // Motor revolutions of movement per loop that still counts as stalled
    private static final int kDeployStallLoops = 10; // Loops (20ms each) below the tolerance before the intake is called stuck
    private static final double kDeployStallErrorTolerance = 0.25; // Motor revolutions still to travel before a lack of movement counts as a stall

    /* ========== DEPLOY ZEROING ========== */
    private static final double kZeroingVoltage = -2; // Volts, pulls the intake inwards against its hard stop
    private static final double kZeroingStallTolerance = 0.01; // Motor revolutions of movement per loop that still counts as stalled
    private static final int kZeroingStallLoops = 10; // Loops (20ms each) below the tolerance before the hard stop is confirmed
    private static final int kZeroingMinLoops = 10; // Loops to let the motor get moving before a lack of movement counts as a stall
    private static final int kZeroingTimeoutLoops = 250; // Loops (~5s) of pulling before giving up

    private double speedIntakeIntake = 0; // Volts // TODO: Intake speed
    private double speedIntakeIdle = 0; // Volts

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.DRIVEBASE_CANBUS);
    private TalonFX intakeLeftMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, RobotConstants.RIO_BUS);
    private TalonFX intakeRightMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeRightMotorID, RobotConstants.RIO_BUS);
    
    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 0);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc",0);
    private LoggedNetworkNumber logDeployKS = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kS", 0);
    private LoggedNetworkNumber logDeployKV = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kV", 0);
    private LoggedNetworkNumber logDeployKA = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kA", 0);
    private LoggedNetworkNumber logDeployKP = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kP", 15);
    private LoggedNetworkNumber logDeployKI = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kI", 0);
    private LoggedNetworkNumber logDeployKD = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kD", 5);
    private LoggedNetworkNumber logDeployKG = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kG", 0);

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut deployVoltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", true);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;

    private double deployTarget = kDeployMinPosition;
    private boolean deployZeroed = false;
    private double deployLastPosition = 0;
    private int deployStallLoops = 0;
    private double zeroingLastPosition = 0;
    private int zeroingStallLoops = 0;
    private int zeroingLoops = 0;

    private IntakeSubsystem() {
        configureDeployMotors();
        configureIntakeMotors();
        startZeroing();
        intakeLeftMotor.setControl(intakeRequest.withOutput(12));
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

        intakeRightMotor.setControl(new Follower(Constants.IntakeConstants.DeviceIDs.kIntakeLeftMotorID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        //applyState();
        runDeploy();
        publishLogs();
    }

    private void runDeploy() {
        if (deployState == DeployState.ZEROING) {
            runZeroing();
            return;
        }

        // Without a zero the encoder reference is meaningless, so don't let the PID chase a target off of it
        if (!deployZeroed) {
            deployMotor.setControl(deployVoltageRequest.withOutput(0));
            return;
        }

        deploySetpoint = deployTarget;
        deployMotor.setControl(deployRequest.withPosition(deploySetpoint));

        checkExtendStall();
    }

    /**
     * Catches the intake stalling on its way out before it reaches the ceiling, and makes wherever it stopped
     * the new target so the PID stops pushing into whatever it hit.
     */
    private void checkExtendStall() {
        double position = deployMotor.getPosition().getValueAsDouble();
        boolean moved = Math.abs(position - deployLastPosition) >= kDeployStallTolerance;
        deployLastPosition = position;

        // Only meaningful while enabled (the motor cannot move when it has no output) and while still on the way out
        boolean extending = DriverStation.isEnabled() && (deployTarget - position) > kDeployStallErrorTolerance;

        if (!extending || moved) {
            deployStallLoops = 0;
            return;
        }

        deployStallLoops++;
        if (deployStallLoops >= kDeployStallLoops) {
            deployTarget = MathUtil.clamp(position, kDeployMinPosition, kDeployMaxPosition);
            deployStallLoops = 0;
        }
    }

    /**
     * Pulls the intake inwards at a low voltage until it stops moving (the hard stop), then calls that position 0.
     * Runs on startup and whenever {@link #startZeroing()} is called.
     */
    private void runZeroing() {
        // Motor output is ignored while disabled, so the intake would never move and the stall check would trip instantly
        if (!DriverStation.isEnabled()) {
            zeroingLoops = 0;
            zeroingStallLoops = 0;
            zeroingLastPosition = deployMotor.getPosition().getValueAsDouble();
            return;
        }

        deploySetpoint = kZeroingVoltage;
        deployMotor.setControl(deployVoltageRequest.withOutput(kZeroingVoltage));

        double position = deployMotor.getPosition().getValueAsDouble();
        boolean stalled = Math.abs(position - zeroingLastPosition) < kZeroingStallTolerance;
        zeroingLastPosition = position;
        zeroingLoops++;

        // Give the motor a moment to get moving before a lack of movement means anything
        if (zeroingLoops < kZeroingMinLoops) return;

        zeroingStallLoops = stalled ? zeroingStallLoops + 1 : 0;

        if (zeroingStallLoops >= kZeroingStallLoops) {
            deployMotor.setPosition(0);
            deployTarget = kDeployMinPosition;
            deployLastPosition = 0;
            deployStallLoops = 0;
            deployZeroed = true;
            deployState = DeployState.STORE;
        } else if (zeroingLoops >= kZeroingTimeoutLoops) {
            // Never found the hard stop, so stop pulling and stay unzeroed until someone re-triggers zeroing
            deployMotor.setControl(deployVoltageRequest.withOutput(0));
            deployState = DeployState.STORE;
            DriverStation.reportWarning("Intake deploy zeroing timed out, intake is disabled until it is re-zeroed", false);
        }
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
        Logger.recordOutput("NewHopper/Intake/Deploy Target", deployTarget);
        Logger.recordOutput("NewHopper/Intake/Deploy Zeroed", deployZeroed);

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

    /** Restarts the zeroing routine. The routine only makes progress while the robot is enabled. */
    public void startZeroing() {
        deployZeroed = false;
        zeroingLoops = 0;
        zeroingStallLoops = 0;
        zeroingLastPosition = deployMotor.getPosition().getValueAsDouble();
        deployState = DeployState.ZEROING;
    }

    /** Moves the deploy target by the given number of motor revolutions, clamped to the deploy range. */
    public void nudgeDeployTarget(double delta) {
        if (!deployZeroed) return;
        deployTarget = MathUtil.clamp(deployTarget + delta, kDeployMinPosition, kDeployMaxPosition);
    }

    public double getDeployTarget() {
        return deployTarget;
    }

    public boolean isDeployZeroed() {
        return deployZeroed;
    }

    public void setPosition(double pos) {
        deployMotor.setControl(deployRequest.withPosition(pos).withFeedForward(0));
    }

    public void holdPosition() {
        deployMotor.setControl(deployRequest.withPosition(0));
    }

    // public void setPositionVoltage(double voltage) {
    //     deployMotor.setControl(new VoltageOut(voltage));
    // }

    public InstantCommand setAngleStateCommand(DeployState state) {
        return new InstantCommand(() -> setDeployState(state));
    }

    public InstantCommand setIntakeStateCommand(IntakeState state) {
        return new InstantCommand(() -> setIntakeState(state));
    }

    public InstantCommand nudgeDeployTargetCommand(double delta) {
        return new InstantCommand(() -> nudgeDeployTarget(delta));
    }

    public Command startZeroingCommand() {
        return new InstantCommand(() -> startZeroing()).ignoringDisable(true);
    }
}
