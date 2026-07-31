package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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
        EJECT,
        BRAKE,
        IDLE
    }

    private DeployState deployState = DeployState.ZEROING;
    private IntakeState intakeState = IntakeState.IDLE;

    private double angleDeployStore = 0; // Motor revolutions // TODO: Store angle
    private double angleDeployIntake = -17.798; // Motor revolutions // TODO: Intake angle

    /* ========== DEPLOY TARGET ========== */
    // Positive revolutions send the intake out, matching the Inverted setting in configureDeployMotors()
    public static final double kDeployExtendPosition = 18.2; // Motor revolutions, intake extended out
    // Motor revolutions, intake pulled in. Deliberately clear of the 0 hard stop, which sits inconsistently.
    // 1 was too tight, the mechanism binds around 1.16 and the motor sat there stalling against it
    public static final double kDeployStorePosition = 2;
    private static final double kDeployMinPosition = 0; // Motor revolutions, the hard stop zeroing finds
    private static final double kDeployMaxPosition = kDeployExtendPosition;
    private static final double kDeployStallTolerance = 0.01; // Motor revolutions of movement per loop that still counts as stalled
    private static final int kDeployStallLoops = 10; // Loops (20ms each) below the tolerance before the intake is called stuck
    private static final double kDeployStallErrorTolerance = 0.25; // Motor revolutions still to travel before a lack of movement counts as a stall
    private static final double kDeployExtendedTolerance = 1; // Motor revolutions short of the extend target that still counts as extended

    /* ========== INTAKE WHEELS ========== */
    private static final double kIntakeVoltage = 8; // Volts, positive pulls balls in
    private static final double kIntakeEjectVoltage = -3; // Volts, negative spits them back out

    /* ========== DEPLOY ZEROING ========== */
    private static final double kZeroingVoltage = -4; // Volts. Negative pulls the intake in, per the Inverted setting in configureDeployMotors()
    private static final double kZeroingStallTolerance = 0.01; // Motor revolutions of movement per loop that still counts as stalled
    private static final int kZeroingStallLoops = 6; // Loops (20ms each) below the tolerance before the hard stop is confirmed
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
    // Accel and Veloc must stay non-zero. For non-Expo Motion Magic 0 does not mean "unlimited" (that
    // only applies to Motion Magic Expo), it means the generated profile never advances, so the motor
    // holds where it is no matter what position is commanded.
    //
    // These two, NOT kP, are what set how long a move takes. kP only decides how hard the motor tracks
    // the profile they generate, so raising it to speed up travel just adds overshoot.
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 60);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc", 30);

    // kV/kS carry the profile so the PID only has to correct what is left over. With every feedforward
    // at 0 the gains had to generate all of the motion from error alone, which lags on the way out and
    // rings at the setpoint. kV is roughly 12V / free speed in rot/s.
    private LoggedNetworkNumber logDeployKS = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kS", 0.15);
    private LoggedNetworkNumber logDeployKV = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kV", 0.12);
    private LoggedNetworkNumber logDeployKA = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kA", 0);
    private LoggedNetworkNumber logDeployKP = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kP", 4);
    private LoggedNetworkNumber logDeployKI = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kI", 0);
    // Volts per rot/s of error derivative. The old 5 turned a 1 rot/s wobble into a 5V swing, which fed
    // encoder noise straight back into the output
    private LoggedNetworkNumber logDeployKD = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kD", 0.2);
    private LoggedNetworkNumber logDeployKG = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kG", 0);

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut deployVoltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeLeftRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRightRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", true);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;

    private double deployTarget = kDeployStorePosition;
    private boolean deployZeroed = false;
    private boolean wasTeleopEnabled = false;
    private double deployLastPosition = 0;
    private int deployStallLoops = 0;
    private double zeroingLastPosition = 0;
    private int zeroingStallLoops = 0;
    private int zeroingLoops = 0;

    private IntakeSubsystem() {
        configureDeployMotors();
        configureDeploySignals();
        configureIntakeMotors();
        startZeroing();

        // A Talon holds onto its last control request, even across a code restart, so the rollers would
        // otherwise pick straight back up at whatever they were last commanded the moment we enable.
        // Both are commanded now that the right roller is no longer a follower of the left
        intakeLeftMotor.setControl(intakeLeftRequest.withOutput(0));
        intakeRightMotor.setControl(intakeRightRequest.withOutput(0));
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

    /**
     * Pins the signals worth plotting to a diagnostic rate. Position and velocity already stream fast,
     * but the closed loop signals default far slower, which is not enough to see a setpoint oscillation
     * in a hoot log. Closed Loop Reference is the profiled setpoint Motion Magic is actually chasing,
     * which is what separates "gains are too hot" from "the profile itself is slow".
     * Drop this back down once the deploy is tuned, it is CAN bandwidth.
     */
    private void configureDeploySignals() {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            deployMotor.getPosition(),
            deployMotor.getVelocity(),
            deployMotor.getMotorVoltage(),
            deployMotor.getStatorCurrent(),
            deployMotor.getClosedLoopReference(),
            deployMotor.getClosedLoopError()
        );
    }

    private void configureIntakeMotors() {
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        // The rollers face each other, so the same positive command has to turn them opposite ways.
        // Each direction is set on its own motor rather than through a Follower, so the two spin
        // directions are stated outright instead of depending on follower alignment semantics
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeLeftMotor.getConfigurator().apply(intakeMotorConfig);

        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeRightMotor.getConfigurator().apply(intakeMotorConfig);
    }

    @Override
    public void periodic() {
        //applyState();

        // Zero on entering teleop, and only teleop. Zeroing drives the intake into its hard stop, which
        // is not something to do partway through an auto routine
        boolean teleopEnabled = DriverStation.isTeleopEnabled();
        if (teleopEnabled && !wasTeleopEnabled) startZeroing();
        wasTeleopEnabled = teleopEnabled;

        runDeploy();
        runIntakeWheels();
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
     * Drives the rollers off the deploy position. Neither direction is any use with the intake stored,
     * and rejecting while stored would just fire balls into the frame, so both are gated on being out.
     */
    private void runIntakeWheels() {
        boolean extended = isExtended();

        switch (intakeState) {
            case INTAKE:
                intakeSetpoint = extended ? kIntakeVoltage : 0;
                break;
            case EJECT:
                intakeSetpoint = extended ? kIntakeEjectVoltage : 0;
                break;
            case BRAKE:
            case IDLE:
                intakeSetpoint = 0;
                break;
        }

        intakeLeftMotor.setControl(intakeLeftRequest.withOutput(intakeSetpoint));
        intakeRightMotor.setControl(intakeRightRequest.withOutput(intakeSetpoint));
    }

    /** True once the deploy has actually arrived at the extended target, not merely been commanded there. */
    public boolean isExtended() {
        return deployZeroed
            && deployMotor.getPosition().getValueAsDouble() >= kDeployExtendPosition - kDeployExtendedTolerance;
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
     * Pulls the intake inwards at a low voltage until it stops moving (the hard stop), stops, and calls
     * that position 0. Kicked off by every enable.
     */
    private void runZeroing() {
        // Only ever run this in teleop, and motor output is ignored while disabled anyway, which would
        // leave the intake still and trip the stall check instantly
        if (!DriverStation.isTeleopEnabled()) {
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
            // Stop pulling before anything else, then call where it stopped the zero
            deployMotor.setControl(deployVoltageRequest.withOutput(0));
            deployMotor.setPosition(0);
            deployTarget = kDeployStorePosition;
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

        // Superseded by runIntakeWheels(), which is what actually drives the rollers. Do not re-enable
        // applyState() without deleting this half, or the two will fight over the same motors every loop
        switch (intakeState) {
            case IDLE:
                intakeSetpoint = speedIntakeIdle;
                intakeLeftMotor.setControl(intakeLeftRequest.withOutput(intakeSetpoint));
                break;
            case INTAKE:
                intakeSetpoint = speedIntakeIntake;
                intakeLeftMotor.setControl(intakeLeftRequest.withOutput(intakeSetpoint));
                break;
            case EJECT:
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
            Logger.recordOutput("NewHopper/Intake/Deploy Stator Current", deployMotor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Closed Loop Reference", deployMotor.getClosedLoopReference().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Closed Loop Error", deployMotor.getClosedLoopError().getValueAsDouble());
            
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

    /** Sets the deploy target in motor revolutions, clamped to the deploy range. Ignored until the intake has zeroed. */
    public void setDeployTarget(double position) {
        if (!deployZeroed) return;
        deployTarget = MathUtil.clamp(position, kDeployMinPosition, kDeployMaxPosition);
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

    /** Sends the intake out to the extended target. The rollers pick up on their own once it arrives. */
    public InstantCommand extendCommand() {
        return new InstantCommand(() -> {
            setDeployTarget(kDeployExtendPosition);
            setIntakeState(IntakeState.INTAKE);
        });
    }

    /** Brings the intake back in to the stored target, and stops the rollers with it. */
    public InstantCommand storeCommand() {
        return new InstantCommand(() -> {
            setDeployTarget(kDeployStorePosition);
            setIntakeState(IntakeState.IDLE);
        });
    }
}
