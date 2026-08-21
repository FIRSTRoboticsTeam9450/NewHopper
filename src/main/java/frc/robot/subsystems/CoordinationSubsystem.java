package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.VisionSubsystem.ZoneState;

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
    private ZoneState previousZone = ZoneState.SHOOTING_ZONE;
    private ZoneState currentZone = ZoneState.SHOOTING_ZONE;

    private boolean driverAllowingShooting = false;
    private boolean checkZeroing = false;
    private boolean update = false;
    private boolean overrideTrench = false;
    private boolean hardOverrideTrench = false;
    private boolean forceTrench = false;
    private boolean rejecting = false;
    private boolean previousGoodToShoot = goodToFire();

    private boolean isIntaking = false;
    private boolean shiftingTurret = true;
    public CoordinationSubsystem() {}

    @Override
    public void periodic() {
        currentZone = getCurrentZone();
        
        if(checkZeroing) {
            setState(AbsoluteStates.INIT, false);
            checkZeroing = !isZeroingDone();
        }
        applyZone();
        publishLogs();
    }

    private void applyZone() {
        if(currentZone != previousZone || update || previousGoodToShoot != goodToFire()) {
            previousZone = currentZone;
            update = false;
            previousGoodToShoot = goodToFire();

            switch (currentZone) {
                case SHOOTING_ZONE:
                    if (forceTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else if(goodToFire()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    }
                    break;
                    
                case FERRY_ZONE_TOP:
                    if (forceTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else if(goodToFire()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    }
                    break; 
                    
                case FERRY_ZONE_BOTTOM:
                    if (forceTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else if(goodToFire()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    }
                    break;

                case SOFT_TRENCH_DEADZONE:
                    // if(!overrideTrench && !hardOverrideTrench) {
                    //     setState(AbsoluteStates.TRENCH, false);
                    // } else {
                    //     setState(AbsoluteStates.SHOOTING, false);
                    // }
                    if (forceTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else if(goodToFire()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    }

                    break;

                case HARD_TRENCH_DEADZONE:
                    if(!hardOverrideTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else {
                        setState(AbsoluteStates.SHOOTING, false);
                    }
                    break;
                case CLIMB_DEADZONE:
                    SidesweeperSubsystem.getInstance().setRampState(RampState.IDLE);
                    SidesweeperSubsystem.getInstance().setSidesweeperState(SidesweeperState.IDLE);
                    break;

                case BACK_SHOOT:
                    setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    break;
                case SHOOT_OVER_NET:
                    if (forceTrench) {
                        setState(AbsoluteStates.TRENCH, false);
                    } else if(goodToFire()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
                    }
                    break;
                default:
                    return;
            }
        }
    
    }

    public void applyState() {
        TurretState turretState = TurretState.DISABLED;
        ShooterState shooterVelocState = ShooterState.NOT_SHOOTING;
        AngleState shooterHoodState = AngleState.DISABLED;
        TransferState transferState = TransferState.IDLE;
        IntakeState intakeWheelState = IntakeState.IDLE;
        DeployState intakeDeployState = DeployState.DEPLOY;
        SidesweeperState sidesweepState = SidesweeperState.IDLE;
        RampState rampState = RampState.IDLE;
        if(isIntaking) {
            // intakeDeployState = DeployState.AGITATE;
            intakeWheelState = IntakeState.INTAKE;
        }
        switch(currentState) {
            case INIT:
                turretState = TurretState.INIT;
                shooterVelocState = ShooterState.NOT_SHOOTING;
                shooterHoodState = AngleState.ZEROING;
                intakeDeployState = DeployState.ZEROING;
                break;
            case PREPARING_FOR_SHOOTING:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                break;
            case SHOOTING:
                transferState = TransferState.TRANSFER;
                rampState = RampState.TRANSFER;
                sidesweepState = SidesweeperState.TRANSFER;
                shooterHoodState = AngleState.AIMING;
                turretState = TurretState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                intakeDeployState = DeployState.AGITATE;
                intakeWheelState = IntakeState.INTAKE;
                break;
            case STORING:
                turretState = TurretState.STORING;
                shooterHoodState = AngleState.STORING;
                break;
            case TRENCH:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.STORING;
                shooterVelocState = ShooterState.SHOOTING;
                break;
            
        }
        if (rejecting) {
            rampState = RampState.EJECT;
            intakeDeployState = DeployState.DEPLOY;
            intakeWheelState = IntakeState.EJECT;
            transferState = TransferState.EJECT;
            sidesweepState = SidesweeperState.EJECT;
        }
        TurretSubsystem.getInstance().setState(turretState);
        ShooterSubsystem.getInstance().setState(shooterVelocState, shooterHoodState);
        IntakeSubsystem.getInstance().setDeployState(intakeDeployState);
        IntakeSubsystem.getInstance().setIntakeState(intakeWheelState);
        SidesweeperSubsystem.getInstance().setSidesweeperState(sidesweepState);
        HopperTransferSubsystem.getInstance().setTransferState(transferState);
        SidesweeperSubsystem.getInstance().setRampState(rampState);

    }

    public void publishLogs() {
        
        Logger.recordOutput("NewHopper/Coord/Intakeing", isIntaking);
        Logger.recordOutput("NewHopper/Coord/Firing", driverAllowingShooting);
        // Logger.recordOutput("NewHopper/Coord/Supercharged", fastTransfer);
        Logger.recordOutput("NewHopper/Coord/Zeroing", checkZeroing);
        Logger.recordOutput("NewHopper/Coord/Current State", currentState);
        // Logger.recordOutput("NewHopper/Coord/Driver Allows Shooting", driverAllowingShooting);
        // Logger.recordOutput("NewHopper/Coord/Subsystems good to fire?", areShooterAndTurretReady());
        // Logger.recordOutput("NewHopper/Coord/Is Shooter Ready?", isShooterReady());
        // Logger.recordOutput("NewHopper/Coord/Is Turret Ready?", isTurretReady());
        // Logger.recordOutput("NewHopper/Coord/OverridingTrench?", overrideTrench);
    }
    
    public boolean isZeroingDone() {
        boolean zeroingDone = ShooterSubsystem.getInstance().getCurrentAngleState() != AngleState.ZEROING 
        && IntakeSubsystem.getInstance().getDeployState() != DeployState.ZEROING;
        Logger.recordOutput("NewHopper/Coord/zeroingDonebyBoth", zeroingDone);
        if(zeroingDone) {
            System.out.println("so you did it.");
            setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
            checkZeroing = false;
        }
        return zeroingDone;
    }
    
    public ZoneState getCurrentZone() {
        return VisionSubsystem.getInstance().getCurrentZone();
    }

    public boolean isShooterReady() {
        return ShooterSubsystem.getInstance().isShooterReady();
        // return true;
    }

    public boolean isTurretReady() {
        return TurretSubsystem.getInstance().getCurrentState() == TurretState.AIMING && TurretSubsystem.getInstance().isTurretGoodToFire();

    }

    public boolean areShooterAndTurretReady() {
        return (isShooterReady() && isTurretReady());
    }
    
    public boolean goodToFire() {
        // return driverAllowingShooting;
        return (driverAllowingShooting && areShooterAndTurretReady());
    }

    public Command setFireCommand() {
        return new InstantCommand(() -> driverAllowingShooting = !driverAllowingShooting);
    }

    public Command setFireCommand(boolean value) {
        return new InstantCommand(() -> driverAllowingShooting = value);
    }

    public Command setIntakeCommand() {
        return new InstantCommand(() -> isIntaking = !isIntaking)
            .andThen(new InstantCommand(() -> CoordinationSubsystem.getInstance().applyState()));
    }

    public Command setIntakeCommand(boolean value) {
        return new InstantCommand(() -> isIntaking = value)
            .andThen(new InstantCommand(() -> CoordinationSubsystem.getInstance().applyState()));
    }


    public Command setOverrideCommand(boolean value) {
        return new InstantCommand(() -> overrideTrench = value)
            .alongWith(new InstantCommand(() -> update = true));
    }

    public Command setRejectingCommand(boolean value) {
        return new InstantCommand(() -> rejecting = value)
            .alongWith(new InstantCommand(() -> update = true));
    }

    public Command setForceTrenchCommand() {
        return new InstantCommand(() -> forceTrench = !forceTrench)
        .alongWith(new InstantCommand(() -> update = true));
    }

    public void setState(AbsoluteStates state, boolean forceStateChange) {
        currentState = state;
        applyState();
    }

    public Command setStateCommand(AbsoluteStates state, boolean forceStateChange) {
        return new InstantCommand(() -> setState(state, forceStateChange));
    }
    
    public void setIntake(boolean intaking) {
        isIntaking = intaking;
    }

    public Command increaseManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust());
        return Commands.either(new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust()), new InstantCommand(() -> ShooterSubsystem.getInstance().addManualVelocityShift()), (() -> shiftingTurret));
    }

    public Command decreaseManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust());
        return Commands.either(new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust()), new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualVelocityShift()), (() -> shiftingTurret));
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
