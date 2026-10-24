package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.PIDVals;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
// import frc.robot.subsystems.vision.VisionSubsystem.ZoneState;

public class ShooterSubsystem extends SubsystemBase {

    // States
    public enum AngleState {
        DISABLED,
        ZEROING,
        AIMING,
        DEADZONE,
        STORING,
        TUNING,
        MANUAL
    }

    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING,
        HOLD_IN,
        MANUAL
    }

    private AngleState currentAngleState = AngleState.ZEROING;
    private ShooterState currentShooterState = ShooterState.SHOOTING;

    private double targetHoodHeight = 0;
    private double targetHoodAngleOverride = 0;
    private double targetShooterVeloc = 0;

    private double hoodAngleMax = 7.5; //old: 14.3
    private double hoodAngleMin = 0; //old: 3.5
    private double hoodFerryMotorRev = 7.35;
    private double degreesPerRev = ShooterConstants.HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 15;
    private double distanceAwayToUseLowDegree = 1.2;
    private double distanceAwayToUseHighDegree = 1.78;

    // Motors TODO: FIX THE MOTOR IDS BEING SWAPPED ONCE THE LEFT MOTOR COMES BACK
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    // Hood Motion Magic parameters
    private LoggedNetworkNumber logHoodMMVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Velocity", 800);
    private LoggedNetworkNumber logHoodMMAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Acceleration", 800);
    private double hoodMMVelocity = logHoodMMVeloc.get();
    private double hoodMMAcceleration = logHoodMMAccel.get();

    private PIDVals hoodPID = new PIDVals("/Tuning/Shooter/Angle/",0.118,0.15,0,30,0,0.1,0.2); 
    /*
     * 
        kS,
        kV,
        kA,
        kP,
        kI,
        kD,
        kG
     */

    private LoggedNetworkNumber logHoodMotorRevTarget = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Target", 13);
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters

    // private PIDVals outtakePID = new PIDVals("/Tuning/Shooter/Outtake/",0.12696,0.11984,0.023895,0.6,0.0,0.0,0.0); // kP = 0.6
    private PIDVals outtakePID = new PIDVals("/Tuning/Shooter/Outtake/",0.13482,0.115,0.020888,0.4,0.0,0.0,0.0); // kP = 0.6

    private LoggedNetworkNumber logOuttakeVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 2.0);
    private LoggedNetworkNumber logOuttakeVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 9450);
    private double outtakeVKFF = logOuttakeVKFF.get();
    private final int outtakeVSlot = 0;
    private final MotionMagicVelocityVoltage outtakeVRequest;

    // Tuning velocity table
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);
    private LoggedNetworkBoolean logIsTuningShooterVelocity = new LoggedNetworkBoolean("/Tuning/Shooter/Outtake/TuningShooterVelocity?", false);

    // Flags
    private boolean isTuningShooterVelocity = logIsTuningShooterVelocity.get();
    private boolean isOverrideAngle = false;
    private boolean isZeroingDone = false;
    private boolean hasSwitchedBack = false;
    private boolean hasReachedShooterVelocity = false;
    private boolean swappedMotors = false;

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);
    private double manualHoodSetpoint = 0;
    private double manualShift = 0; // For manually adjusting shooter velocity on the fly, added to getShooterVeloc()
    private double manualHoodShift = 0;
    private double wheelWareAdjustment = 0;
    private double previousHoodTimestamp = 0.0;
    private double previousFlywheelTimestamp = 0.0;

    // Timer
    private Timer switchSlotBackTimer = new Timer();
    int slotNum = 0;

    // Data tables
    InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shootingDistanceToVelocity = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shootingDistanceToAngle = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ferryingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingOverNetDistanceToVelocity = new InterpolatingDoubleTreeMap();

    // Cached Values
    private double cachedDistanceToTarget = 0.0;

    // ----------------------------- CONSTRUCTOR ---------------------------- //
    public ShooterSubsystem() {
        swappedMotors = false;
        configureAngleMotors();
        configureShooterMotors();
        addData();
        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot).withEnableFOC(RobotConstants.ENABLE_FOC);

        m_sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null,        // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
                    (state) -> Logger.recordOutput("NewHopper/Shooter/Outtake/SysID/State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> shooterLeftMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
                    null,
                    this
                )
            );

    }

    public void addData() { // 73 is max velocity for max hood to make it from the furthest point in shooter

        shootingDistanceToVelocity.put(1.24, 27.25 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(1.96, 27.75 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(3.06, 31.25 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(4.06, 35.00 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(4.53, 37.15);
        shootingDistanceToVelocity.put(4.74, 40.5 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(5.73, 42.0 + wheelWareAdjustment);
        //Made up
        shootingDistanceToVelocity.put(6.75, 44.0 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(7.75, 46.0 + wheelWareAdjustment);

        shootingDistanceToTime.put(1.56, 1.02);
        shootingDistanceToTime.put(2.16, 1.01);
        shootingDistanceToTime.put(3.06, 1.05);
        shootingDistanceToTime.put(4.06, 1.14);
        shootingDistanceToTime.put(4.74, 1.33);
        shootingDistanceToTime.put(5.61, 1.27);
        shootingDistanceToTime.put(6.0, 1.2);
        shootingDistanceToTime.put(6.5, 1.25);
        shootingDistanceToTime.put(7.0, 1.3);
        // shootingDistanceToTime.put(8.0, 1.71);
        // shootingDistanceToTime.put(12.0, 2.0);

        // shootingDistanceToAngle.put(1.4, 0.0 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(2.0, .7 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(3.16, 4.185 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(3.6, 2.44 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(3.61, 2.44 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(3.75, 2.89 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(4.14, 3.97 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(5.0, 4.97 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(5.2, 4.97 + wheelWareAdjustment); // 65
        // shootingDistanceToAngle.put(5.82, 7.97 + wheelWareAdjustment); // 65

        //OLD SHOOTER TABLE
        // shootingDistanceToAngle.put(1.334, 1.37); // 65
        // shootingDistanceToAngle.put(2.027, 2.8); // 65
        // shootingDistanceToAngle.put(2.3, 3.35); // 65
        // shootingDistanceToAngle.put(2.713, 6.9); // 65
        // shootingDistanceToAngle.put(3.3, 10.17); // 65
        // shootingDistanceToAngle.put(3.455, 8.9); // 65
        // shootingDistanceToAngle.put(3.57, 9.5); // 65
        
        // shootingDistanceToAngle.put(3.74, 6.577); // 73
        // shootingDistanceToAngle.put(3.97, 7.577); // 73
        // shootingDistanceToAngle.put(4.242, 10.4);
        // shootingDistanceToAngle.put(4.434, 11.4); // 73
        // shootingDistanceToAngle.put(4.8, 11.7); // 73
        // shootingDistanceToAngle.put(4.9, 13.00); // 73
        // shootingDistanceToAngle.put(5.4, 13.2); // 73
        // shootingDistanceToAngle.put(4.434, 8.59); // 73

        
        // shootingDistanceToVelocity.put(6.2, 51.5 + wheelWareAdjustment - .25);
        // shootingDistanceToVelocity.put(5.684, 49.75 + wheelWareAdjustment + .5);
        // shootingDistanceToVelocity.put(5.095, 46.0 + wheelWareAdjustment + .75);
        // shootingDistanceToVelocity.put(4.101, 42.5 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(3.465, 41.0 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(3.0, 38.75 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(2.4, 36.5 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(1.73, 33.55 + wheelWareAdjustment);
        // shootingDistanceToVelocity.put(1.1, 30.0 + wheelWareAdjustment);

        // shootingDistanceToTime.put(2.462, 1.29);

        // shootingDistanceToTime.put(1.71, 1.1925); // Video 2!
        // shootingDistanceToTime.put(2.399, 1.11); // Video 1!
        // shootingDistanceToTime.put(3.271, 1.026); // Video 4!
        // shootingDistanceToTime.put(4.28, 1.2); // Video 5!
        // shootingDistanceToTime.put(5.28, 1.25); // Video 1 Nicholas
        // shootingDistanceToTime.put(5.8, 1.2025); // Video 1 Mark 
        // shootingDistanceToTime.put(1.21, 1.25);
        // shootingDistanceToTime.put(1.81, 1.25);
        // shootingDistanceToTime.put(2.872, 1.1);
        // shootingDistanceToTime.put(3.951, 1.292);
        // shootingDistanceToTime.put(4.43, 1.2725);
        // shootingDistanceToTime.put(4.83, 1.23);
        // shootingDistanceToTime.put(5.61, 1.25);
        // shootingDistanceToTime.put(6.2, 1.25);
        // shootingDistanceToTime.put(5.095, 1.4525);
        // shootingDistanceToTime.put(4.101, 1.374);
        // shootingDistanceToTime.put(3.465, 1.3325);
        // shootingDistanceToTime.put(3.0, 1.235);
        // shootingDistanceToTime.put(2.4, 1.1383);
        // shootingDistanceToTime.put(1.73, 1.032);
        // shootingDistanceToTime.put(1.1, 0.956);



        // Ferrying Data Velocity
        // ferryingDistanceToVelocity.put(4.59, 34.5);
        // ferryingDistanceToVelocity.put(4.962, 35.72);
        // ferryingDistanceToVelocity.put(7.622, 42.03);
        // ferryingDistanceToVelocity.put(11.26, 52.0);
        // ferryingDistanceToVelocity.put(16.26, 62.75);

        ferryingDistanceToVelocity.put(4.59, 28.5 + 2.5);
        ferryingDistanceToVelocity.put(4.962, 29.72 + 2.5);
        ferryingDistanceToVelocity.put(7.622, 36.03 + 8);
        ferryingDistanceToVelocity.put(11.26, 46.0);
        ferryingDistanceToVelocity.put(14.26, 104.0);

        // Ferrying Data Time
        ferryingDistanceToTime.put(4.59, 1.41);
        ferryingDistanceToTime.put(11.26, 1.96);


    }

    public void configureAngleMotors() {
        angleMotorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;

        hoodPID.setMotorPID0(angleMotorConfig);

        hoodPID.setMotorPID1(angleMotorConfig);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logHoodMMAccel.get())
                                                               .withMotionMagicCruiseVelocity(logHoodMMVeloc.get());
        angleMotorConfig.MotionMagic = mmConfigs;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodAngleMax; // Hood angle limit
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        angleMotor.getConfigurator().apply(angleMotorConfig);
    }

   private void configureShooterMotors() {
        shooterMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterMotorsConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        shooterMotorsConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimit = 70;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        outtakePID.setMotorPID0(shooterMotorsConfig);

        shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();

        shooterLeftMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterRightMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterRightMotor.setControl(new Follower(shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean autoZeroing() {
        if(!DriverStation.isAutonomousEnabled()) {
            if(Math.abs(angleMotor.getMotorVoltage().getValueAsDouble()) > 0.4 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) < 0.05) {
                angleMotor.setPosition(0);
                angleMotor.setVoltage(0);
                setState(currentShooterState, AngleState.STORING);
                return true;
            }
        }
        else {
            angleMotor.setPosition(0);
            setState(currentShooterState, AngleState.STORING);
            RobotContainer.coordSub.setState(AbsoluteStates.TRENCH, false);
            return true;
        }
        return false; // TODO CHANGE BACK TO FALSE
    }

    public void zeroEncoder() {
        if (DriverStation.isDisabled()) {
            angleMotor.setPosition(0);
        }
    }

    @Override
    public void periodic() {
        // cachedDistanceToTarget = RobotContainer.visionSub.getDistanceToTarget(RobotContainer.visionSub.getAdjustedCurrentTarget());

        if (!isZeroingDone) {
            currentAngleState = AngleState.ZEROING;
            isZeroingDone = autoZeroing();
        }

        // if(!shooterLeftMotor.isConnected() && !swappedMotors) {
        //     swappedMotors = true;
        //     shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
        //     shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
        // }

        updateReachedShooterVelocityLogic();

        publishLogs();

        if(switchSlotBackTimer.get() > 2 && currentAngleState == AngleState.AIMING && !hasSwitchedBack) {
            hasSwitchedBack = true;
            slotNum = 0;
            hoodMMRequest.withVelocity(hoodMMVelocity)
                         .withAcceleration(hoodMMAcceleration);
        }
        applyState();
    }

    public void publishLogs() {
        Logger.recordOutput("NewHopper/Shooter/Hood/currentState", currentAngleState);
        Logger.recordOutput("NewHopper/Shooter/Hood/angleSetpointMotorRevs", targetHoodHeight + manualHoodShift);
        // Logger.recordOutput("NewHopper/Shooter/Hood/angleSetpointDegrees", targetHoodHeight * degreesPerRev + degreesToZero);
        Logger.recordOutput("NewHopper/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Hood/Left Supply Amps", angleMotor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Hood/Left Stator Amps", angleMotor.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Hood/current degrees", angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero);
        // Logger.recordOutput("NewHopper/Shooter/Hood/Is at Goal?", isAngleWithinTolerance());

        Logger.recordOutput("NewHopper/Shooter/Outtake/currentState", currentShooterState);
        Logger.recordOutput("NewHopper/Shooter/Outtake/velocitySetpoint", targetShooterVeloc + manualShift);
        // Logger.recordOutput("NewHopper/Shooter/Outtake/Left Supply Amps", shooterLeftMotor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Outtake/Left Stator Amps", shooterLeftMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("NewHopper/Shooter/Outtake/Left Position", shooterLeftMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("NewHopper/Shooter/Outtake/Left current velocity", shooterLeftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("NewHopper/Shooter/Outtake/Left motor voltage", shooterLeftMotor.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Outtake/Left motor Position", shooterLeftMotor.getPosition().getValueAsDouble());
        // Logger.recordOutput("NewHopper/Shooter/Outtake/Is at Goal?", isVelocityWithinTolerance());

        Logger.recordOutput("NewHopper/Shooter/Outtake/Manual Adjustment", manualShift);


        if (logDebuggingToggle.get()) { // To get rid of loop overruns


            Logger.recordOutput("NewHopper/Shooter/Time To Shoot", getShootingDistanceToTime(cachedDistanceToTarget));

            // Hood logs
            Logger.recordOutput("NewHopper/Shooter/Hood/Took target", hoodAngleFunction(cachedDistanceToTarget));
            Logger.recordOutput("NewHopper/Shooter/Hood/voltage", angleMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("NewHopper/Shooter/Hood/Request Accel", hoodMMRequest.Acceleration);
            Logger.recordOutput("NewHopper/Shooter/Hood/Request Veloc", hoodMMRequest.Velocity);

            // Logger.recordOutput("NewHopper/Shooter/Hood/Override hood setpoint", targetHoodAngleOverride);
            // Logger.recordOutput("NewHopper/Shooter/Hood/Overriding hood", isOverrideAngle);

            // Outtake logs
            // Logger.recordOutput("NewHopper/Shooter/Outtake/Right motor voltage", shooterRightMotor.getMotorVoltage().getValueAsDouble());
            // Logger.recordOutput("NewHopper/Shooter/Outtake/TargetOverrideVelocity", logTuningShooterVelocity.get());
            // Logger.recordOutput("NewHopper/Shooter/Outtake/Tuning?", isTuningShooterVelocity);

            // Logger.recordOutput("NewHopper/Shooter/Outtake/Right Motor Position", shooterRightMotor.getPosition().getValueAsDouble());
        }
    }

    public void applyState() {
        switch (currentShooterState) {
            case SHOOTING:
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(getShooterVeloc()));
                break;

            case NOT_SHOOTING:
                shooterLeftMotor.setVoltage(0);
                break;

            case HOLD_IN:
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(-10));
                break;
            case MANUAL:
                shooterLeftMotor.setVoltage(MathUtil.clamp(manualShift, 0, 12));

            default:
                break;
        }
        switch (currentAngleState) {
            case ZEROING:
                // if (TurretSubsystem.getInstance().getCurrentState() == TurretSubsystem.TurretState.INIT && TurretSubsystem.getInstance().isAtProfiledGoal(10)) {
                    // angleMotor.setVoltage(-0.6);
                // } else {
                // }
                if(!DriverStation.isAutonomousEnabled()) {
                    angleMotor.setControl(new VoltageOut(-0.60));
                }
                else {
                    angleMotor.setControl(new VoltageOut(0));
                }

                break;

            case AIMING:
                angleMotor.setControl(hoodMMRequest.withPosition(getShooterAngle()).withSlot(slotNum));
                break;

            case STORING:
                // if (TurretSubsystem.getInstance().getCurrentState() == TurretSubsystem.TurretState.STORING && TurretSubsystem.getInstance().isAtProfiledGoal(15)) {
                    angleMotor.setControl(hoodMMRequest.withPosition(0).withSlot(slotNum));
                // }
                break;

            case TUNING:
                double targetAngle = MathUtil.clamp(logHoodMotorRevTarget.get(), hoodAngleMin, hoodAngleMax);
                angleMotor.setControl(hoodMMRequest.withPosition(targetAngle).withSlot(slotNum));
                break;
            case MANUAL:
                targetHoodHeight = manualHoodSetpoint + manualHoodShift;
                angleMotor.setControl(hoodMMRequest.withPosition(manualHoodSetpoint + manualHoodShift).withSlot(slotNum));

            default:
                break;
        }
    }


    // --------------------------------------- GETTERS --------------------------------------- //
    public double getShooterVeloc() {
        if(!isTuningShooterVelocity) {
            cachedDistanceToTarget = RobotContainer.visionSub.getDistanceToTarget(RobotContainer.visionSub.getAdjustedFutureTarget(Timer.getFPGATimestamp() - previousFlywheelTimestamp));
            previousFlywheelTimestamp = Timer.getFPGATimestamp();
            switch (RobotContainer.coordSub.getCurrentZone()) {

                case SHOOTING_ZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 5.6) {
                    //     targetShooterVeloc = 44.5;
                    // }
                    // else if(cachedDistanceToTarget > 4.5) {
                    //     targetShooterVeloc = 38.15;
                    // }
                    // else if(cachedDistanceToTarget > 3.3) {
                    //     targetShooterVeloc = 38;
                    // }
                    // else {
                    //     targetShooterVeloc = 32;
                    // }
                    // if(cachedDistanceToTarget > 4.5) {
                    //     targetShooterVeloc = 73;
                    // }
                    // else if(cachedDistanceToTarget > 3.6) {
                    //     targetShooterVeloc = 69;
                    // }
                    // else {
                    //     targetShooterVeloc = 61;
                    // }
                    break;

                case CLIMB_DEADZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 4.5) {
                    //     targetShooterVeloc = 40;
                    // }
                    // else if(cachedDistanceToTarget > 3.6) {
                    //     targetShooterVeloc = 32;
                    // }
                    // else {
                    //     targetShooterVeloc = 32;
                    // }
                    // if(cachedDistanceToTarget > 4.5) {
                    //     targetShooterVeloc = 73;
                    // }
                    // else if(cachedDistanceToTarget > 3.6) {
                    //     targetShooterVeloc = 69;
                    // }
                    // else {
                    //     targetShooterVeloc = 61;
                    // }
                    break;

                case FERRY_ZONE_TOP:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 90; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;

                case FERRY_ZONE_BOTTOM:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 90; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;

                case FERRY_ZONE_MID:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 60; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;

                case SOFT_TRENCH_DEADZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case HARD_TRENCH_DEADZONE:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case BACK_SHOOT:
                    targetShooterVeloc = 70;
                    break;
                case SHOOT_OVER_NET:
                    targetShooterVeloc = 90;
                    break;

                default:
                    break;
            }
            if(RobotContainer.visionSub.getUsingCustomTarget()) {
                targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
            }
        }
        else {
            targetShooterVeloc = logTuningShooterVelocity.get();
        }
        targetShooterVeloc = MathUtil.clamp(targetShooterVeloc, 0, 100);
        return targetShooterVeloc + manualShift;
    }

    public double getShooterAngle() {
        if(!isOverrideAngle) {
            double distance = RobotContainer.visionSub.getDistanceToTarget(RobotContainer.visionSub.getAdjustedFutureTarget(Timer.getFPGATimestamp() - previousHoodTimestamp));
            previousHoodTimestamp = Timer.getFPGATimestamp();
            switch (RobotContainer.coordSub.getCurrentZone()) {
                case SHOOTING_ZONE:
                    // targetHoodHeight = shootingDistanceToAngle.get(distance);
                    targetHoodHeight = hoodAngleFunction(distance);
                    break;
                case CLIMB_DEADZONE:
                    targetHoodHeight = hoodAngleFunction(distance);
                    // targetHoodHeight = shootingDistanceToAngle.get(distance);
                    break;

                case FERRY_ZONE_TOP:
                    targetHoodHeight = hoodAngleFunction(distance);
                    // targetHoodHeight = hoodFerryMotorRev;
                    break;

                case FERRY_ZONE_BOTTOM:
                    targetHoodHeight = hoodAngleFunction(distance);
                    // targetHoodHeight = hoodFerryMotorRev;
                    break;

                case FERRY_ZONE_MID:
                    targetHoodHeight = hoodFerryMotorRev;
                    break;

                case HARD_TRENCH_DEADZONE:
                    setAngleState(AngleState.STORING);
                    break;

                case SOFT_TRENCH_DEADZONE:
                    targetHoodHeight = hoodAngleFunction(distance);
                    break;

                case BACK_SHOOT:
                    targetHoodHeight = MathUtil.clamp(getAsRevs(20), hoodAngleMin, hoodAngleMax);
                    break;

                case SHOOT_OVER_NET:
                    targetHoodHeight = hoodAngleFunctionShootOverNet(distance);
                    break;

                default:
                    return 0;
            }
            if(RobotContainer.visionSub.getUsingCustomTarget()) {
                targetHoodHeight = hoodAngleFunction(distance);
            }

        }
        else {
            targetHoodHeight = targetHoodAngleOverride;
        }
        return targetHoodHeight + manualHoodShift;

    }

    public double getShootingDistanceToTime(double distance) {
        return shootingDistanceToTime.get(distance);
    }

    public double getFerryingDistanceToTime(double distance) {
        return ferryingDistanceToTime.get(distance);
    }

    public double hoodAngleFunction(double distance) {
        // double wantedAngle = (-60/(distance + 0.8)) + 47;
        // double wantedAngle = (-200/(distance + 6)) + 39;
        // double wantedAngle = -(200/(distance + 5)) + 47;
        double wantedAngle = (-180/(distance + 3)) + 60;

        double wantedRevs = getAsRevs(wantedAngle);
        if(distance <= distanceAwayToUseLowDegree) {
            wantedRevs = 0;
        }
        // if(distance >= distanceAwayToUseHighDegree && RobotContainer.coordSub.getCurrentZone() != ZoneState.SHOOTING_ZONE) {
        //     wantedRevs = hoodAngleMax;
        // }
        Logger.recordOutput("NewHopper/Shooter/Hood/wantedAngle", wantedAngle);
        return MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax);
    }

    public double hoodAngleFunctionShootOverNet(double distance) {
        // double wantedAngle = (-120/(distance + 4.4)) + 45;
        double wantedAngle = (-600/(distance + 28)) + 39;
        double wantedRevs = getAsRevs(wantedAngle);
        if(distance <= distanceAwayToUseLowDegree) {
            wantedRevs = 0;
        }
        // if(distance >= distanceAwayToUseHighDegree && RobotContainer.coordSub.getCurrentZone() != ZoneState.SHOOTING_ZONE) {
        //     wantedRevs = hoodAngleMax;
        // }
        return MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax);
    }

    public void setManualHoodSetpoint(double setpoint) {
        manualHoodSetpoint = setpoint;
    }

    public double getHoodAngleDegrees() {
        return angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero; // add + 15 if you want true angle
    }

    public double getAsRevs(double degrees) {
        return (degrees-degreesToZero) / degreesPerRev;
    }

    public AngleState getCurrentAngleState() {
        return currentAngleState;
    }

    // Checks
    public boolean isAngleWithinTolerance() {
        return isAngleWithinTolerance(0.75);
    }

    public boolean isAngleWithinTolerance(double tolerance) {
        return Math.abs(targetHoodHeight + manualHoodShift - angleMotor.getPosition().getValueAsDouble()) < tolerance;
    }

    public boolean isVelocityWithinTolerance() {
        return isVelocityWithinTolerance(5);
    }

    public boolean isVelocityWithinTolerance(double tolerance) {
        double shooterVeloc = shooterLeftMotor.getVelocity().getValueAsDouble();
        if (targetShooterVeloc >= 75) {
            return true;
        }
        return Math.abs(targetShooterVeloc + manualShift - shooterVeloc) < tolerance;
    }

    public boolean isShooterReady() { //TODO: Add back at some point ¯\_(ツ)_/¯
        if(!hasReachedShooterVelocity) {
            return (isAngleWithinTolerance() && isVelocityWithinTolerance());
        }
        // return false;
        return (isAngleWithinTolerance() && isVelocityWithinTolerance(20)); // TODO: PROB WHAT IS CAUSING THE PAUSES the is velocity within tolerance 20
        // return (isAngleWithinTolerance());
        // return true;
    }

    private void updateReachedShooterVelocityLogic() {
        if (currentShooterState != ShooterState.SHOOTING) {
            hasReachedShooterVelocity = false;
        } else {
            if (Math.abs(shooterLeftMotor.getVelocity().getValueAsDouble() - getShooterVeloc()) < 3) {
                hasReachedShooterVelocity = true;
            }
        }
    }

    // -------------------------------------- SETTERS -------------------------------------- //

    /**
     * Set shooter state
     * @param wantedShooterState
     * @param wantedAngleState
     */
    public void setState(ShooterState wantedShooterState, AngleState wantedAngleState) {
        if(currentAngleState == AngleState.ZEROING || currentAngleState == AngleState.STORING) {
            hoodMMRequest.withVelocity(300)
                         .withAcceleration(300);

            switchSlotBackTimer.restart();
            hasSwitchedBack = false;
        }
        if(wantedAngleState == AngleState.ZEROING) {
            isZeroingDone = false;
        }
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public Command setStateCommand(ShooterState wantedShooterState, AngleState wantedAngleState) {
        return new InstantCommand(() -> setState(wantedShooterState, wantedAngleState));
    }

    public Command setShooterStateCommand(ShooterState wantedShooterState) {
        return setStateCommand(wantedShooterState, currentAngleState);
    }

    public Command setAngleState(AngleState wantedAngleState) {
        return setStateCommand(currentShooterState, wantedAngleState);
    }

    public void setOverrideModeAngle(boolean isOverrideAngle) {
        this.isOverrideAngle = isOverrideAngle;
    }

    public void setOverrideModeShooter() {
        this.isTuningShooterVelocity = true;
    }

    public void setOverrideHoodAngle(double targetHoodAngleOverride) {
        this.targetHoodAngleOverride = MathUtil.clamp(getAsRevs(targetHoodAngleOverride), 0, hoodAngleMax);
    }

    public void addManualVelocityShift() {
        manualShift += 0.5;
    }

    public void addManualVelocityShift(double value) {
        manualShift += value;
    }

    public void subtractManualVelocityShift() {
        manualShift -= 0.5;
    }

    public void addManualHoodShift() {
        manualHoodShift += 0.5;
    }

    public void addManualHoodShift(double value) {
        manualHoodShift += value;
    }

    public void subtractManualHoodShift() {
        manualHoodShift -= 0.5;
    }

    public void resetManualHoodShift() {
        manualHoodShift = 0;
    }

    public void resetManualVelocityShift() {
        manualShift = 0;
    }

    // -------------------------------------------- UPDATE BASED ON LOGS -------------------------------------------- //
    public void updateShooterConstants() {
        boolean updateOuttakeVals = false;
        boolean updateAngleVals = false;
        boolean updateSetpoints = false;

        updateSetpoints = (isTuningShooterVelocity != logIsTuningShooterVelocity.get());
        if(updateSetpoints) {
            isTuningShooterVelocity = logIsTuningShooterVelocity.get();
        }

        if (outtakePID.updateMotorPID(shooterMotorsConfig) || shooterMotorsConfig.MotionMagic.MotionMagicAcceleration != logOuttakeVAccel.get() || outtakeVKFF != logOuttakeVKFF.get()) {

            outtakeVKFF = logOuttakeVKFF.get();

            shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();

            configureShooterMotors();
        }

        if (hoodPID.updateMotorPID(angleMotorConfig) || angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logHoodMMVeloc.get() || angleMotorConfig.MotionMagic.MotionMagicAcceleration != logHoodMMAccel.get()) {

            configureAngleMotors();

            hoodMMVelocity = logHoodMMVeloc.get();
            hoodMMAcceleration = logHoodMMAccel.get();

            hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withSlot(slotNum);//.withEnableFOC(RobotConstants.ENABLE_FOC);
        }

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(angleMotor);
        arrList.add(shooterLeftMotor);
        arrList.add(shooterRightMotor);
        return arrList;
    }

}
