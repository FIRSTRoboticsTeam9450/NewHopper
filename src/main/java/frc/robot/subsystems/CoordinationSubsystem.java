package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperTransferSubsystem.TransferState;
import frc.robot.subsystems.IntakeSubsystem.DeployState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TurretSubsystem.TurretState;

public class CoordinationSubsystem extends SubsystemBase {
    private static CoordinationSubsystem INSTANCE;

    public enum AbsoluteState {
        INIT,
        STORING,
        PREPARING_FOR_SHOOTING,
        SHOOTING,
        TRENCH
    }

    private AbsoluteState currentState;

    public CoordinationSubsystem() {}

    @Override
    public void periodic() {
    }

    public void applyState() {
        TurretState turretState = TurretState.DISABLED;
        ShooterState shooterVelocState = ShooterState.IDLE;
        AngleState angleState = AngleState.STORE;
        TransferState transferState = TransferState.IDLE;
        IntakeState intakeWheelState = IntakeState.IDLE;
        DeployState intakeDeployState = DeployState.STORE;

        switch(currentState) {
            case INIT:
                turretState = TurretState.DISABLED;
                shooterVelocState = ShooterState.IDLE;
                angleState = AngleState.STORE;
                transferState = TransferState.IDLE;
                intakeWheelState = IntakeState.IDLE;
                intakeDeployState = DeployState.STORE;
                break;
            case PREPARING_FOR_SHOOTING:
                break;
            case SHOOTING:
                break;
            case STORING:
                break;
            case TRENCH:
                break;
            default:
                break;
            
        }
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new CoordinationSubsystem();
        return INSTANCE;
    }
}
