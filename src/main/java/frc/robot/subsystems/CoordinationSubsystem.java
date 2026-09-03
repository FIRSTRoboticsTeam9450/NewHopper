package frc.robot.subsystems;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperTransferSubsystem.TransferState;
import frc.robot.subsystems.intake.IntakeSubsystem.DeployState;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem.AngleState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.sidesweeper.SidesweeperSubsystem.RampState;
import frc.robot.subsystems.sidesweeper.SidesweeperSubsystem.SidesweeperState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.subsystems.vision.VisionSubsystem.ZoneState;

public class CoordinationSubsystem extends SubsystemBase {

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
                case SHOOTING_ZONE, FERRY_ZONE_TOP, FERRY_ZONE_BOTTOM, SHOOT_OVER_NET:
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
                    RobotContainer.sidesweeperSub.setRampState(RampState.IDLE);
                    RobotContainer.sidesweeperSub.setSidesweeperState(SidesweeperState.IDLE);
                    break;

                case BACK_SHOOT:
                    setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
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
        RobotContainer.turretSub.setState(turretState);
        RobotContainer.shooterSub.setState(shooterVelocState, shooterHoodState);
        RobotContainer.intakeSub.setDeployState(intakeDeployState);
        RobotContainer.intakeSub.setIntakeState(intakeWheelState);
        RobotContainer.sidesweeperSub.setSidesweeperState(sidesweepState);
        RobotContainer.transferSub.setTransferState(transferState);
        RobotContainer.sidesweeperSub.setRampState(rampState);

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
        boolean zeroingDone = RobotContainer.shooterSub.getCurrentAngleState() != AngleState.ZEROING
        && RobotContainer.intakeSub.getDeployState() != DeployState.ZEROING;
        Logger.recordOutput("NewHopper/Coord/zeroingDonebyBoth", zeroingDone);
        if(zeroingDone) {
            System.out.println("so you did it.");
            setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
            checkZeroing = false;
        }
        return zeroingDone;
    }

    public ZoneState getCurrentZone() {
        return RobotContainer.visionSub.getCurrentZone();
    }

    public boolean isShooterReady() {
        return RobotContainer.shooterSub.isShooterReady();
        // return true;
    }

    public boolean isTurretReady() {
        return RobotContainer.turretSub.getCurrentState() == TurretState.AIMING && RobotContainer.turretSub.isTurretGoodToFire();

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
            .andThen(new InstantCommand(this::applyState));
    }

    public Command setIntakeCommand(boolean value) {
        return new InstantCommand(() -> isIntaking = value)
            .andThen(new InstantCommand(this::applyState));
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
        return Commands.either(new InstantCommand(RobotContainer.turretSub::decreaseManualAdjust), new InstantCommand(RobotContainer.shooterSub::addManualVelocityShift), (() -> shiftingTurret));
    }

    public Command decreaseManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust());
        return Commands.either(new InstantCommand(RobotContainer.turretSub::increaseManualAdjust), new InstantCommand(RobotContainer.shooterSub::subtractManualVelocityShift), (() -> shiftingTurret));
    }

    public Command resetManualAdjust() {
        // return new InstantCommand(() -> TurretSubsystem.getInstance().resetManualAdjust());
        return Commands.either(new InstantCommand(RobotContainer.turretSub::resetManualAdjust), new InstantCommand(RobotContainer.shooterSub::resetManualVelocityShift), (() -> shiftingTurret));
    }

    public Command setManualTurret(boolean value){
        return new InstantCommand(() -> shiftingTurret = value);
    }

}
