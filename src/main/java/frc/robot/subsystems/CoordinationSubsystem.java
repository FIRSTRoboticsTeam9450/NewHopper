package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperTransferSubsystem.TransferState;
import frc.robot.subsystems.IntakeSubsystem.DeployState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
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
        TurretState turretState = TurretState.IDLE;
        ShooterState shooterVelocState = ShooterState.IDLE;
        TransferState transferState = TransferState.IDLE;
        IntakeState intakeWheelState = IntakeState.IDLE;
        DeployState intakeDeployState = DeployState.STORE;
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new CoordinationSubsystem();
        return INSTANCE;
    }
}
