package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoordinationSubsystem extends SubsystemBase {
    private static CoordinationSubsystem INSTANCE;

    public enum AbsoluteState {
    }

    private AbsoluteState currentState;

    public CoordinationSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new CoordinationSubsystem();
        return INSTANCE;
    }
}
