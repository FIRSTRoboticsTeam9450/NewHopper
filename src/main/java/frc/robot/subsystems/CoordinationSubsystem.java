package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperTransferSubsystem.TransferState;
import frc.robot.subsystems.IntakeSubsystem.DeployState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SidesweeperSubsystem.RampState;
import frc.robot.subsystems.SidesweeperSubsystem.SidesweeperState;
import frc.robot.subsystems.TurretSubsystem.TurretState;

public class CoordinationSubsystem extends SubsystemBase {
    private static CoordinationSubsystem INSTANCE;
    public enum AbsoluteStates {
        INIT,
        STORING,
        PREPARING_FOR_SHOOTING,
        SHOOTING,
        TRENCH
    }

    private AbsoluteStates currentState = AbsoluteStates.INIT;
    private boolean isIntaking = false;
    private boolean shiftingTurret = true;
    public CoordinationSubsystem() {}

    @Override
    public void periodic() {
    }

    public void applyState() {
        TurretState turretState = TurretState.DISABLED;
        ShooterState shooterVelocState = ShooterState.NOT_SHOOTING;
        AngleState angleState = AngleState.STORING;
        TransferState transferState = TransferState.IDLE;
        IntakeState intakeWheelState = IntakeState.IDLE;
        DeployState intakeDeployState = DeployState.STORE;
        if(isIntaking) {
            intakeDeployState = DeployState.DEPLOY;
            intakeWheelState = IntakeState.INTAKE;
        }
        SidesweeperState sidesweepState = SidesweeperState.IDLE;
        RampState rampState = RampState.IDLE;
        switch(currentState) {
            case INIT:
                turretState = TurretState.INIT;
                shooterVelocState = ShooterState.NOT_SHOOTING;
                angleState = AngleState.ZEROING;
                intakeDeployState = DeployState.ZEROING;
                break;
            case PREPARING_FOR_SHOOTING:
                break;
            case SHOOTING:
                transferState = TransferState.TRANSFER;
                rampState = RampState.TRANSFER;
                sidesweepState = SidesweeperState.TRANSFER;
                angleState = AngleState.AIMING;
                turretState = TurretState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                if (!isIntaking) {
                    intakeDeployState = DeployState.AGITATE;
                }
                break;
            case STORING:
                break;
            case TRENCH:
                break;
            default:
            
                break;
            
        }
        
        TurretSubsystem.getInstance().setState(turretState);
        ShooterSubsystem.getInstance().setState(shooterVelocState, angleState);
        IntakeSubsystem.getInstance().setDeployState(intakeDeployState);
        IntakeSubsystem.getInstance().setIntakeState(intakeWheelState);
        HopperTransferSubsystem.getInstance().setTransferState(transferState);
        SidesweeperSubsystem.getInstance().setSidesweeperState(sidesweepState);
        SidesweeperSubsystem.getInstance().setRampState(rampState);

    }

    public void setState(AbsoluteStates state, boolean forceStateChange) {
        currentState = state;
        applyState();
    }
    public void setIntake(boolean intaking) {
        isIntaking = intaking;
    }

    public Command increaseManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust());
        return Commands.either(new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust()), new InstantCommand(() -> ShooterSubsystem.getInstance().addManualVelocityShift()), (() -> shiftingTurret));
    }

    public Command decreaseManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust());
        return Commands.either(new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust()), new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualVelocityShift()), (() -> shiftingTurret));
    }

    public Command resetManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().resetManualAdjust());
        return Commands.either(new InstantCommand(() -> TurretSubsystem.getInstance().resetManualAdjust()), new InstantCommand(() -> ShooterSubsystem.getInstance().resetManualVelocityShift()), (() -> shiftingTurret));
    }

    public Command setManualTurret(boolean value){
        return new InstantCommand(() -> shiftingTurret = value);
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new CoordinationSubsystem();
        return INSTANCE;
    }
}
