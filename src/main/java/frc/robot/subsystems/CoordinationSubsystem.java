package frc.robot.subsystems;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.HubTracker;
import frc.robot.util.HubTracker.Shift;

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
    private boolean justZeroed = true;
    private boolean update = false;
    private boolean overrideTrench = false;
    private boolean hardOverrideTrench = false;
    private boolean forceTrench = true;
    private boolean rejecting = false;
    private boolean previousGoodToShoot = goodToFire();
    private boolean previousDriverAllowingShooting = getdriverAllowingShooting();
    private boolean previousRejecting = rejecting;
    private boolean previousLaser = hasLotsOfFuel();
    //private boolean previousBump = onBump();
    private boolean continueAgitating = false;

    private boolean isIntaking = false;
    private boolean shiftingTurret = false;
    private boolean isAgitate = false;
    private boolean isFastAgitate = false;
    private boolean isIntakeOut = true;
    private boolean isSlowTransfer = false;
    private boolean isAutoAgitateOff = false;
    // private double turretBumpAngle = -159.0;
    public CoordinationSubsystem() {}

    @Override
    public void periodic() {
        currentZone = getCurrentZone();

        if(checkZeroing) {
            setState(AbsoluteStates.INIT, false);
            checkZeroing = !isZeroingDone();
        }
        applyZone();
        // publishLogs();
    }

    private void applyZone() {
        if(currentZone != previousZone || update || previousGoodToShoot != goodToFire() || previousDriverAllowingShooting != getdriverAllowingShooting()
        || previousRejecting != rejecting || previousLaser != hasLotsOfFuel()) { // || previousBump != onBump()) {
            // if(currentZone == ZoneState.SHOOTING_ZONE && previousZone == ZoneState.FERRY_ZONE_BOTTOM) {
            //     turretBumpAngle = -159.0;
            // }
            // else if(currentZone == ZoneState.SHOOTING_ZONE && previousZone == ZoneState.FERRY_ZONE_TOP){
            //     turretBumpAngle = -159.0;

            // }
            previousZone = currentZone;
            update = false;
            previousGoodToShoot = goodToFire();
            previousDriverAllowingShooting = getdriverAllowingShooting();
            previousRejecting = rejecting;
            previousLaser = hasLotsOfFuel();
            // previousBump = onBump();

            switch (currentZone) {
                case SHOOTING_ZONE, FERRY_ZONE_TOP, FERRY_ZONE_BOTTOM, SHOOT_OVER_NET:
                    if(getdriverAllowingShooting()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.TRENCH, false);
                    }
                    break;

                case SOFT_TRENCH_DEADZONE:
                    // if(!overrideTrench && !hardOverrideTrench) {
                    //     setState(AbsoluteStates.TRENCH, false);
                    // } else {
                    //     setState(AbsoluteStates.SHOOTING, false);
                    // }
                    if(getdriverAllowingShooting()) {
                        setState(AbsoluteStates.SHOOTING, false);
                    } else {
                        setState(AbsoluteStates.TRENCH, false);
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
        if(!isIntakeOut) {
            intakeDeployState = DeployState.STORE;
            intakeWheelState = IntakeState.OFF;
        }
        if(isAgitate) {
            intakeDeployState = DeployState.AGITATE;
        }
        switch(currentState) {
            case INIT:
                turretState = TurretState.INIT;
                shooterVelocState = ShooterState.NOT_SHOOTING;
                shooterHoodState = AngleState.ZEROING;
                intakeDeployState = DeployState.ZEROING;
                // justZeroed = false;
                break;
            case PREPARING_FOR_SHOOTING:
                if (onBump()) turretState = TurretState.BUMP;
                else turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                break;
            case SHOOTING:
                shooterHoodState = AngleState.AIMING;
                if (onBump()) turretState = TurretState.BUMP;
                else turretState = TurretState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                intakeWheelState = IntakeState.INTAKE;
                if(areShooterAndTurretReady()) {
                    transferState = TransferState.TRANSFER;
                    rampState = RampState.TRANSFER;
                    sidesweepState = SidesweeperState.TRANSFER;
                    if(!isAutoAgitateOff) {
                        if (hasLotsOfFuel() && !continueAgitating) {
                             intakeDeployState = DeployState.DEPLOY;
                        }
                        else {
                            intakeDeployState = DeployState.AGITATE_FAST;
                            continueAgitating = true;

                        }
                    }
                    if(isSlowTransfer) {
                        transferState = TransferState.SLOW_TRANSFER;
                    }
                    if(currentZone == ZoneState.FERRY_ZONE_TOP || currentZone == ZoneState.FERRY_ZONE_BOTTOM) {
                        transferState = TransferState.SLOW_TRANSFER;
                        intakeDeployState = DeployState.DEPLOY;   
                    }
                }
                break;
            case STORING:
                turretState = TurretState.STORING;
                shooterHoodState = AngleState.STORING;
                break;
            case TRENCH:
                if (onBump()) turretState = TurretState.BUMP;
                else turretState = TurretState.AIMING;
                shooterHoodState = AngleState.STORING;
                shooterVelocState = ShooterState.SHOOTING;
                break;

        }
        if(isFastAgitate) {
            intakeDeployState = DeployState.AGITATE_FAST;
        }
        if(isIntaking) {
            intakeDeployState = DeployState.DEPLOY;
            intakeWheelState = IntakeState.INTAKE;
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
        Logger.recordOutput("NewHopper/Match Data/Our Hub Active?", HubTracker.isActive());
        Logger.recordOutput("NewHopper/Match Data/We Win Auto?", DriverStation.getAlliance().isPresent() && HubTracker.getAutoWinner().isPresent() ? DriverStation.getAlliance().get() == HubTracker.getAutoWinner().get() : false);
        Logger.recordOutput("NewHopper/Match Data/Time Left Until Shift Change", HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get() : -1);
        Logger.recordOutput("NewHopper/Match Data/Shift", HubTracker.getCurrentShift().isPresent() ? HubTracker.getCurrentShift().get() : Shift.AUTO);

        Logger.recordOutput("NewHopper/Coord/Intakeing", isIntaking);
        Logger.recordOutput("NewHopper/Coord/Firing", driverAllowingShooting);
        // Logger.recordOutput("NewHopper/Coord/Supercharged", fastTransfer);
        Logger.recordOutput("NewHopper/Coord/Zeroing", checkZeroing);
        Logger.recordOutput("NewHopper/Coord/Current State", currentState);
        // Logger.recordOutput("NewHopper/Coord/Driver Allows Shooting", driverAllowingShooting);
        Logger.recordOutput("NewHopper/Coord/Subsystems good to fire?", areShooterAndTurretReady());
        Logger.recordOutput("NewHopper/Coord/Is Shooter Ready?", isShooterReady());
        Logger.recordOutput("NewHopper/Coord/justZeroed", justZeroed);
        Logger.recordOutput("NewHopper/Coord/onBump", onBump());

        // Logger.recordOutput("NewHopper/Coord/OverridingTrench?", overrideTrench);
    }

    public boolean isZeroingDone() {
        boolean zeroingDone = RobotContainer.shooterSub.getCurrentAngleState() != AngleState.ZEROING
        && RobotContainer.intakeSub.getDeployState() != DeployState.ZEROING;
        Logger.recordOutput("NewHopper/Coord/zeroingDonebyBoth", zeroingDone);
        if(zeroingDone) {
            // System.out.println("so you did it.");
            setState(AbsoluteStates.TRENCH, false);
            checkZeroing = false;
        }
        return zeroingDone;
    }

    public boolean hasLotsOfFuel() {
         return RobotContainer.transferSub.canSeeFuel();
    }

    public boolean onBump() {
        return false;
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

    public boolean getdriverAllowingShooting() {
        return driverAllowingShooting;
    }
    public Command setIntakeCommand() {
        return new InstantCommand(() -> isIntaking = !isIntaking)
            .andThen(new InstantCommand(this::applyState));
    }

    public Command setSlowTransfer() {
        return new InstantCommand(() -> isSlowTransfer = !isSlowTransfer)
        .andThen(new InstantCommand(this::applyState));
    }
    public Command setSlowTransfer(boolean value) {
        return new InstantCommand(() -> isSlowTransfer = value)
        .andThen(new InstantCommand(this::applyState));
    }

    public Command setAgitate() {
        return new InstantCommand(() -> isAgitate = !isAgitate)
        .andThen(new InstantCommand(this::applyState));
    }
    public Command setAgitate(boolean value) {
        return new InstantCommand(() -> isAgitate = value)
        .andThen(new InstantCommand(this::applyState));
    }

    public Command setFastAgitate() {
        return new InstantCommand(() -> isFastAgitate = !isFastAgitate)
        .andThen(new InstantCommand(this::applyState));
    }
    public Command setFastAgitate(boolean value) {
        return new InstantCommand(() -> isFastAgitate = value)
        .andThen(new InstantCommand(this::applyState));
    }

    public Command setAutoAgitateOff() {
        return new InstantCommand(() -> isAutoAgitateOff = !isAutoAgitateOff)
        .andThen(new InstantCommand(this::applyState));
    }
    public Command setAutoAgitateOff(boolean value) {
        return new InstantCommand(() -> isAutoAgitateOff = value)
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
    public Command setForceTrenchCommand(boolean value) {
        return new InstantCommand(() -> forceTrench = value)
        .alongWith(new InstantCommand(() -> update = true));
    }

    public Command setIntakeOut() {
        return new InstantCommand(() -> isIntakeOut = !isIntakeOut)
        .alongWith(new InstantCommand(() -> update = true));
    }

    public Command setIntakeOut(boolean value) {
        return new InstantCommand(() -> isIntakeOut = value)
        .alongWith(new InstantCommand(() -> update = true));
    }

    public void setState(AbsoluteStates state, boolean forceStateChange) {
        currentState = state;
        if(currentState != AbsoluteStates.INIT && state == AbsoluteStates.INIT) {
            justZeroed = true;
        }
        if(currentState != AbsoluteStates.SHOOTING && state == AbsoluteStates.SHOOTING) {
            continueAgitating = false;
        }
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
