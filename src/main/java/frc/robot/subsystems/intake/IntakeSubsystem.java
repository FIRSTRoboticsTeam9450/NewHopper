package frc.robot.subsystems.intake;

import frc.robot.util.PIDVals;
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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import static frc.robot.RobotContainer.coordSub;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.shooter.ShooterSubsystem.AngleState;

public class IntakeSubsystem extends SubsystemBase {

    public enum DeployState {
        ZEROING,
        DEPLOY,
        STORE,
        AGITATE,
        AGITATE_FAST,
        TUNING
    }

    public enum IntakeState {
        INTAKE,
        BRAKE,
        IDLE,
        EJECT,
        OFF
    }

    private double speedIntakeIntake = 8; // Volts // TODO: Intake speed
    private double speedIntakeIdle = 0.5; // Volts
    private double speedIntakeOff = 0; // Volts

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.DRIVEBASE_CANBUS);
    private TalonFX intakeLeftMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, RobotConstants.RIO_BUS);
    private TalonFX intakeRightMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeRightMotorID, RobotConstants.RIO_BUS);

    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 400);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc",200);

    private PIDVals deployPID = new PIDVals("/Tuning/Intake/Deploy/",0,0.1,0,60,0,0,0);

    private LoggedNetworkNumber logDeploySetpoint = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Setpoint", 18);
    private LoggedNetworkNumber logAgitationInVolts = new LoggedNetworkNumber("/Tuning/Intake/Deploy/AgitationInVolts", -5);
    private LoggedNetworkNumber logAgitationOutVolts = new LoggedNetworkNumber("/Tuning/Intake/Deploy/AgitationOutVolts", 5);

    private double angleDeployStore = 5; // Motor revolutions // TODO: Store angle
    private double angleDeployIntake = logDeploySetpoint.get(); //17.798; // Motor revolutions // TODO: Intake angle
    private double angleDeployAgitate = angleDeployIntake - 1; // Motor revolutions // TODO: Agitation angle
    private double angleDeployAgitateLimit = 2; // Motor revolutions // TODO: Agitation angle

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", true);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;
    private double tuningDeploySetpoint = 0;
    private double setpointCounter = 0;
    private boolean isZeroingDone = false;
    private double zeroingVolts = 4;
    private double zeroingCount = 0;

    private double deployAgitationTolerance = 0.3;

    private boolean agitateTowardIntake = false;
    private double agitationInVolts = logAgitationInVolts.get();
    private double agitationOutVolts = logAgitationOutVolts.get();
    private Timer agitationTimer = new Timer();
    private Timer waitTimer = new Timer();
    private double lastSwitchTime = 0.0;
    private int agitationStage = 0;
    private double agitationStage0InSetpoint = 18;
    private double agitationStage1InSetpoint = 2;
    private double agitationStage1OutSetpoint = 6;
    private double agitationStage2InSetpoint = 2;
    // private double agitationStage2OutSetpoint = 10;



    public IntakeSubsystem() {
        configureDeployMotors();
        configureIntakeMotors();
        setpointCounter = 0;
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

        deployPID.setMotorPID0(deployMotorConfig);

        deployMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        deployMotor.getConfigurator().apply(deployMotorConfig);
    }

    private void configureIntakeMotors() {
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive = Intake, Negative = Outtake

        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        intakeLeftMotor.getConfigurator().apply(intakeMotorConfig);
        intakeRightMotor.getConfigurator().apply(intakeMotorConfig);

        intakeRightMotor.setControl(new Follower(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        //applyState();
        if(!isZeroingDone){
            coordSub.setIntakeDeployState(DeployState.ZEROING);
            isZeroingDone = isZeroingDone();
            zeroingCount++;
        }

        applyState();
        publishLogs();
    }

    private boolean isZeroingDone(){
        if(coordSub.getIntakeDeployState() == DeployState.ZEROING && Math.abs(deployMotor.getVelocity().getValueAsDouble()) < 0.1 && Math.abs(zeroingVolts) > 1 && Math.abs(deployMotor.getMotorVoltage().getValueAsDouble()) > 0 && zeroingCount > 5){
            if(coordSub.getShooterHoodState() != AngleState.ZEROING) {
                deployMotor.setPosition(0);
                deployMotor.setVoltage(0);
                zeroingVolts = 0;
                zeroingCount = 0;
                coordSub.setIntakeDeployState(DeployState.DEPLOY);
                isZeroingDone = true;
                RobotContainer.coordSub.setState(AbsoluteStates.TRENCH, false);
                return true;
            }
        }
        else if(RobotContainer.coordSub.getIsAuto()) {
            if(RobotContainer.shooterSub.getCurrentAngleState() != AngleState.ZEROING) {
                deployMotor.setPosition(0);
                deployMotor.setVoltage(0);
                zeroingVolts = 0;
                zeroingCount = 0;
                deployState = DeployState.DEPLOY;
                isZeroingDone = true;
                RobotContainer.coordSub.setState(AbsoluteStates.TRENCH, false);
                return true;
            }
        }
        return false;
    }

    private void applyState() {
        switch (coordSub.getIntakeDeployState()) {
            case ZEROING:
                if(!RobotContainer.coordSub.getIsAuto()) {
                    zeroingVolts = 4;
                    deployMotor.setVoltage(-zeroingVolts);
                }
                else {
                    zeroingVolts = 0;
                    deployMotor.setVoltage(zeroingVolts);
                }
                break;
            case DEPLOY:
                deploySetpoint = angleDeployIntake;
                sendSetpoint();
                break;
            case STORE:
                deploySetpoint = angleDeployStore;
                sendSetpoint();
                break;
            case TUNING:
                deploySetpoint = tuningDeploySetpoint;
                sendSetpoint();
                break;
            case AGITATE:
                // if(waitTimer.hasElapsed(1.5)) {
                    // double voltage = agitateTowardIntake
                    //     ? agitationInVolts
                    //     : agitationOutVolts;

                    // deployMotor.setVoltage(voltage);
                    // if (Timer.getFPGATimestamp() - lastSwitchTime >= 0.5
                    // || (deployMotor.getPosition().getValueAsDouble() > angleDeployIntake && voltage > 0)
                    // || (deployMotor.getPosition().getValueAsDouble() < angleDeployAgitateLimit && voltage < 0)) {
                    //     agitateTowardIntake = !agitateTowardIntake;
                    //     lastSwitchTime = Timer.getFPGATimestamp();
                    // }

                    // if(agitationStage == 0) {
                    //     deploySetpoint = agitationStage0InSetpoint;
                    // }
                    // else if(agitationStage == 1) {
                    //     deploySetpoint = agitationStage1InSetpoint;
                    // }
                    // else if(agitationStage == 2) {
                    //     deploySetpoint = agitationStage1OutSetpoint;
                    // }
                    // else if(agitationStage == 3) {
                    //     deploySetpoint = agitationStage2InSetpoint;
                    // }
                    // else if(agitationStage == 3) {
                    //     deploySetpoint = agitationStage2OutSetpoint;
                    // }
                    // else if(agitationStage == 4) {
                    //     deploySetpoint = agitationStage2InSetpoint;
                    // }
                    // else if(agitationStage == 5) {
                    //     deploySetpoint = agitationStage2OutSetpoint;
                    // }
                    deploySetpoint = 4;
                    deployMotor.setControl(deployRequest.withPosition(deploySetpoint));

                    // double deployMotorPosition = deployMotor.getPosition().getValueAsDouble();
                    // if (Math.abs(deployMotorPosition - deploySetpoint) <= deployAgitationTolerance) {
                    //     if(agitationStage == 3) {
                    //         agitationStage = 1;
                    //     }
                    //     agitationStage++;
                    // }

                // }
                // else {
                //     deployMotor.setVoltage(0);
                // }
                break;
            case AGITATE_FAST:
                // if(waitTimer.hasElapsed(1.5)) {
                    // double voltage = agitateTowardIntake
                    //     ? agitationInVolts
                    //     : agitationOutVolts;

                    // deployMotor.setVoltage(voltage);
                    // if (Timer.getFPGATimestamp() - lastSwitchTime >= 0.5
                    // || (deployMotor.getPosition().getValueAsDouble() > angleDeployIntake && voltage > 0)
                    // || (deployMotor.getPosition().getValueAsDouble() < angleDeployAgitateLimit && voltage < 0)) {
                    //     agitateTowardIntake = !agitateTowardIntake;
                    //     lastSwitchTime = Timer.getFPGATimestamp();
                    // }

                    if(agitationStage == 0) {
                        deploySetpoint = agitationStage0InSetpoint;
                    }
                    else if(agitationStage == 1) {
                        deploySetpoint = agitationStage1InSetpoint;
                    }
                    else if(agitationStage == 2) {
                        deploySetpoint = agitationStage1OutSetpoint;
                    }
                    else if(agitationStage == 3) {
                        deploySetpoint = agitationStage2InSetpoint;
                    }

                    double deployMotorPosition = deployMotor.getPosition().getValueAsDouble();
                    if (Math.abs(deployMotorPosition - deploySetpoint) <= deployAgitationTolerance) {
                        if(agitationStage == 3) {
                            agitationStage = 1;
                        }
                        agitationStage++;
                    }
                    deployMotor.setControl(deployRequest.withPosition(deploySetpoint));
                    break;

        }

        switch (coordSub.getIntakeWheelState()) {
            case IDLE:
                intakeSetpoint = speedIntakeIdle;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
                break;
            case OFF:
                intakeSetpoint = speedIntakeOff;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
            case INTAKE:
                intakeSetpoint = speedIntakeIntake;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
                break;
            case BRAKE:
                break;
            case EJECT:
                intakeSetpoint = -6;
                intakeLeftMotor.setControl(intakeRequest.withOutput(intakeSetpoint));
                break;
        }
    }
    private void sendSetpoint() {
        double error = Math.abs(deploySetpoint - deployMotor.getPosition().getValueAsDouble());
        if(error > .2) {
            setpointCounter = 0;
        }
        if(error < IntakeConstants.DEPLOY_TOL) {
            setpointCounter += 1;
        }
        if(setpointCounter > 10 && deploySetpoint < 10){
            deployMotor.setVoltage(0);
        }
        else {
            deployMotor.setControl(deployRequest.withPosition(deploySetpoint));

        }

    }

    // private boolean checkStall() {
    //     // if(intakeLeftMotor.)
    // }
    private void publishLogs() {
        //TODO Move logging of states to coord sub
        Logger.recordOutput("NewHopper/Intake/Angle State", coordSub.getIntakeDeployState());
        Logger.recordOutput("NewHopper/Intake/Intake State", coordSub.getIntakeWheelState());
        Logger.recordOutput("NewHopper/Intake/Deploy Setpoint", deploySetpoint);
        Logger.recordOutput("NewHopper/Intake/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("NewHopper/Intake/Intake Voltage Setpoint", intakeSetpoint);
        Logger.recordOutput("NewHopper/Intake/Intake Left Voltage", intakeLeftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("NewHopper/Intake/Intake Right Voltage (follower)", intakeRightMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("NewHopper/Intake/Agitation/WaitTimer", waitTimer.hasElapsed(2));
        Logger.recordOutput("NewHopper/Intake/Agitation/AgitationTimer", Timer.getFPGATimestamp() - lastSwitchTime);
        Logger.recordOutput("NewHopper/Intake/Agitation/agitateTowardIntake", agitateTowardIntake);
        Logger.recordOutput("NewHopper/Intake/Agitation/AgitationStage", agitationStage);
        Logger.recordOutput("NewHopper/Intake/iszeroingdone", isZeroingDone);
        if (logIntakeDebug.get()) {
            Logger.recordOutput("NewHopper/Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());

            Logger.recordOutput("NewHopper/Intake/Intake Velocity", intakeLeftMotor.getVelocity().getValueAsDouble());
            // Logger.recordOutput("NewHopper/Intake/Intake Position", intakeLeftMotor.getPosition().getValueAsDouble());

            Logger.recordOutput("NewHopper/Intake/Deploy Supply Amps", deployMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Stator Amps", deployMotor.getStatorCurrent().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Intake/Intake Supply Amps", intakeLeftMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Intake Stator Amps", intakeLeftMotor.getStatorCurrent().getValueAsDouble());
        }
    }

    public void updateConstants() {
        angleDeployIntake = logDeploySetpoint.get();
        angleDeployAgitate = angleDeployIntake - 1;
        agitationInVolts = logAgitationInVolts.get();
        agitationOutVolts = logAgitationOutVolts.get();
        if (deployPID.updateMotorPID(deployMotorConfig)) configureDeployMotors();
    }

    public void setDeploySetpoint(double setpoint) {
        tuningDeploySetpoint = setpoint;
    }

    public void updatedDeployState() {

        //None of these conditions should ever be true with the new system

        /*
        if(deployState != state){
            setpointCounter = 0;
        }
        if(deployState != DeployState.AGITATE && state == DeployState.AGITATE) {
            waitTimer.restart();
            deployMotorConfig.MotionMagic.MotionMagicAcceleration = 100;
            deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 12;
            deployMotor.getConfigurator().apply(deployMotorConfig);

            lastSwitchTime = Timer.getFPGATimestamp();
            agitationStage = 0;
        }
        if(deployState == DeployState.AGITATE_FAST && state != DeployState.AGITATE_FAST) {
            waitTimer.restart();
            deployMotorConfig.MotionMagic.MotionMagicAcceleration = 100;
            deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
            deployMotor.getConfigurator().apply(deployMotorConfig);

            lastSwitchTime = Timer.getFPGATimestamp();
            agitationStage = 0;

        }
        if(deployState == DeployState.AGITATE && state != DeployState.AGITATE) {
            deployMotorConfig.MotionMagic.MotionMagicAcceleration = 400;
            deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
            deployMotor.getConfigurator().apply(deployMotorConfig);
        }
        if(deployState == DeployState.AGITATE_FAST && state != DeployState.AGITATE_FAST) {
            deployMotorConfig.MotionMagic.MotionMagicAcceleration = 400;
            deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
            deployMotor.getConfigurator().apply(deployMotorConfig);
        }
        if(state == DeployState.ZEROING && deployState != DeployState.ZEROING) {
            isZeroingDone = false;
            zeroingCount = 0;
            zeroingVolts = 0;
            // System.out.println("Yay its working.");
        }
        deployState = state;*/
    }

    public void setIntakeVolts(double volts) {
        speedIntakeIntake = volts;
    }

}
