package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem INSTANCE;
    
    public enum DeployState {
        ZEROING,
        DEPLOY,
        STORE,
        AGITATE,
        TUNING
    }

    public enum IntakeState {
        INTAKE,
        BRAKE,
        IDLE,
        EJECT
    }

    private DeployState deployState = DeployState.ZEROING;
    private IntakeState intakeState = IntakeState.IDLE;

    private double speedIntakeIntake = 8; // Volts // TODO: Intake speed
    private double speedIntakeIdle = 0; // Volts

    private TalonFX deployMotor = new TalonFX(IntakeConstants.DeviceIDs.kDeployMotorID, RobotConstants.DRIVEBASE_CANBUS);
    private TalonFX intakeLeftMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeLeftMotorID, RobotConstants.RIO_BUS);
    private TalonFX intakeRightMotor = new TalonFX(IntakeConstants.DeviceIDs.kIntakeRightMotorID, RobotConstants.RIO_BUS);
    
    private TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    // TODO: Motion Magic tuning
    private LoggedNetworkNumber logDeployAccel = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Accel", 400);
    private LoggedNetworkNumber logDeployVeloc = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Veloc",200);
    private LoggedNetworkNumber logDeployKS = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kS", 0);
    private LoggedNetworkNumber logDeployKV = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kV", 0.1);
    private LoggedNetworkNumber logDeployKA = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kA", 0);
    private LoggedNetworkNumber logDeployKP = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kP", 60);
    private LoggedNetworkNumber logDeployKI = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kI", 0);
    private LoggedNetworkNumber logDeployKD = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kD", 0);
    private LoggedNetworkNumber logDeployKG = new LoggedNetworkNumber("/Tuning/Intake/Deploy/kG", 0);
    private LoggedNetworkNumber logDeploySetpoint = new LoggedNetworkNumber("/Tuning/Intake/Deploy/Setpoint", 18);

    private LoggedNetworkNumber logDeploySlot =  new LoggedNetworkNumber("/Tuning/Intake/Deploy/Slot", 0);

    private double angleDeployStore = 5; // Motor revolutions // TODO: Store angle
    private double angleDeployIntake = logDeploySetpoint.get(); //17.798; // Motor revolutions // TODO: Intake angle
    private double angleDeployAgitate = angleDeployIntake - 1; // Motor revolutions // TODO: Agitation angle

    private MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private LoggedNetworkBoolean logIntakeDebug = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Intake", true);

    private double deploySetpoint = 0;
    private double intakeSetpoint = 0;
    private double tuningDeploySetpoint = 0;
    private double setpointCounter = 0;
    private boolean isZeroingDone = false;
    
    private double deployAgitationTolerance = 0.3;

    private boolean agitateTowardIntake = false;


    private IntakeSubsystem() {
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

        Slot0Configs slot0Config = new Slot0Configs().withKS(logDeployKS.get())
                                                     .withKV(logDeployKV.get())
                                                     .withKA(logDeployKA.get())
                                                     .withKP(logDeployKP.get())
                                                     .withKI(logDeployKI.get())
                                                     .withKD(logDeployKD.get())
                                                     .withKG(logDeployKG.get())
                                                     .withGravityType(GravityTypeValue.Elevator_Static);
        
        deployMotorConfig.Slot0 = slot0Config;
        
        
        Slot1Configs slot1Config = new Slot1Configs().withKS(logDeployKS.get())
                                                     .withKV(logDeployKV.get())
                                                     .withKA(logDeployKA.get())
                                                     .withKP(logDeployKP.get())
                                                     .withKI(logDeployKI.get())
                                                     .withKD(logDeployKD.get())
                                                     .withKG(logDeployKG.get())
                                                     .withGravityType(GravityTypeValue.Elevator_Static);

        deployMotorConfig.Slot1 = slot1Config;

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
        if(!isZeroingDone){
            deployState = DeployState.ZEROING;
            isZeroingDone = isZeroingDone();
        }

        applyState();
        publishLogs();
    }

    private boolean isZeroingDone(){
        if(Math.abs(deployMotor.getVelocity().getValueAsDouble()) < 0.1 && Math.abs(deployMotor.getMotorVoltage().getValueAsDouble()) > 1){
            deployMotor.setPosition(0);
            deployMotor.setVoltage(0);
            deployState = DeployState.STORE;
            return true;
        }
        return false;
    }

    private void applyState() {
        switch (deployState) {
            case ZEROING:
                deployMotor.setVoltage(-4);
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
                double deploySetpoint = agitateTowardIntake
                        ? angleDeployIntake
                        : angleDeployAgitate;
                
                deployMotor.setControl(deployRequest.withPosition(deploySetpoint).withSlot((int) logDeploySlot.get()));

                double deployMotorPosition = deployMotor.getPosition().getValueAsDouble();
                if (Math.abs(deployMotorPosition - deploySetpoint) <= deployAgitationTolerance) {
                    agitateTowardIntake = !agitateTowardIntake;
                }

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
            deployMotor.setControl(deployRequest.withPosition(deploySetpoint).withSlot(0));

        }

    }
    private void publishLogs() {
        Logger.recordOutput("NewHopper/Intake/Angle State", deployState);
        Logger.recordOutput("NewHopper/Intake/Intake State", intakeState);
        Logger.recordOutput("NewHopper/Intake/Deploy Setpoint", deploySetpoint);
        Logger.recordOutput("NewHopper/Intake/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("NewHopper/Intake/Intake Voltage Setpoint", intakeSetpoint);
        Logger.recordOutput("NewHopper/Intake/Intake Left Voltage", intakeLeftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("NewHopper/Intake/Intake Right Voltage (follower)", intakeRightMotor.getMotorVoltage().getValueAsDouble());

        if (logIntakeDebug.get()) {
            Logger.recordOutput("NewHopper/Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("NewHopper/Intake/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
            
            Logger.recordOutput("NewHopper/Intake/Intake Velocity", intakeLeftMotor.getVelocity().getValueAsDouble());
            // Logger.recordOutput("NewHopper/Intake/Intake Position", intakeLeftMotor.getPosition().getValueAsDouble());
        }
    }

    public void updateConstants() {
        angleDeployIntake = logDeploySetpoint.get();
        angleDeployAgitate = angleDeployIntake - 1;
        
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

    public void setDeploySetpoint(double setpoint) {
        tuningDeploySetpoint = setpoint;
    }

    public void setDeployState(DeployState state) {
        System.err.println(state); // was print(state) originally assumed it should be this
        if(deployState != state){
            setpointCounter = 0;
        }
        deployState = state;
        if(deployState == DeployState.ZEROING) {
            isZeroingDone = false;
        }
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public InstantCommand setAngleStateCommand(DeployState state) {
        return new InstantCommand(() -> setDeployState(state));
    }

    public InstantCommand setIntakeStateCommand(IntakeState state) {
        return new InstantCommand(() -> setIntakeState(state));
    }
}
