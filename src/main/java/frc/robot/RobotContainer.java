// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.log.LogLevel;
import frc.robot.util.log.Logging;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.HopperTransferSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SidesweeperSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.DeployState;
import frc.robot.util.BezierCurve;

public class RobotContainer {
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final VisionSubsystem visionSub = new VisionSubsystem();
    public static final HopperTransferSubsystem transferSub = new HopperTransferSubsystem();
    public static final IntakeSubsystem intakeSub = new IntakeSubsystem();
    public static final ShooterSubsystem shooterSub = new ShooterSubsystem();
    public static final SidesweeperSubsystem sidesweeperSub = new SidesweeperSubsystem();
    public static final TurretSubsystem turretSub = new TurretSubsystem();
    public static final CoordinationSubsystem coordSub = new CoordinationSubsystem();

    
    public static final CommandXboxController DRIVER = new CommandXboxController(RobotConstants.DRIVER_CONTROLLER_PORT);
    public static final CommandXboxController OPERATOR = new CommandXboxController(RobotConstants.OPERATOR_CONTROLLER_PORT);

    public static final boolean enableSysIdKeybinds = false; // Enable SysID keybinds, and disable conflicting keybinds!
        // Currently disables:
        //  Driver:
        //   Reset IMU
        //  Operator:
    
    /**
     * Woojin's Drive: 117.4, 0.054, 91.4, 0.76, 0.1, 0.01
     * Woojin's Rotate: 120.1, 0.145, 92.2, 0.362, 0.035, 0.03
     */
    public static BezierCurve driveBezier = new BezierCurve("drive", 151.0, 0.137, 116, 0.646, 0.1, 0.01);
    public static BezierCurve rotateBezier = new BezierCurve("rotate", 164.8, 0.093, 100, 0.84, 0.035, 0.03);
    
    public static SlewRateLimiter driveXAccelLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter driveYAccelLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter rotationAccelLimiter = new SlewRateLimiter(8.0);
    
    private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static boolean slewMode = false;

    public static double maxSpeed = DEFAULT_MAX_SPEED * 0.2;
    public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE * 1;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(driveBezier.getDeadband())
            .withRotationalDeadband(rotateBezier.getDeadband())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("NewHopper/Auto Chooser");

    public static final Logging testLog = new Logging("NewHopper/", LogLevel.MINIMAL);

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();
        configureAutos();
        testLog.addHandler(new Logging.Handler(Pose2d.class) {
            @Override
            public void handle(String path, Object o) {
                Pose2d pose = (Pose2d) o;
                Logger.recordOutput(path, pose);
            }
        });
        testLog.addLoggedObject(turretSub);
        testLog.addLoggedObject(visionSub);
    }

    private void configureBindings() {
        /* ========== SWERVE KEYBINDS ========== */
        // Drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // double driveX = driveXAccelLimiter.calculate(-driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed);
                // double driveY = driveYAccelLimiter.calculate(-driveBezier.getOutput(DRIVER.getLeftX())  * maxSpeed);
                double driveX = DRIVER.getLeftY();
                double driveY = DRIVER.getLeftX();

                boolean xNegative = driveX < 0;
                boolean yNegative = driveY < 0;

                driveX = xNegative ? (Math.pow(driveX, 2) * -1.0) : Math.pow(driveX, 2);
                driveY = yNegative ? (Math.pow(driveY, 2) * -1.0) : Math.pow(driveY, 2);

                double driveRotate = rotationAccelLimiter.calculate(-rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate);
                double joyX = -DRIVER.getLeftX();
                double joyY = DRIVER.getLeftY();
                double speed = driveBezier.getOutput(Math.hypot(joyX, joyY)) * maxSpeed;
                double rotation = getRotation(joyY, joyX);
                double xUsed = speed * Math.cos(rotation);
                double yUsed = speed * Math.sin(rotation);
                
                // double rotate = -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate;
                double rotate = DRIVER.getRightX();
                boolean xRotatePositive = rotate > 0;
                rotate = xRotatePositive ? (Math.pow(rotate, 2) * -1.0) * maxAngularRate : Math.pow(rotate, 2) * maxAngularRate;

                return drive.withVelocityX(slewMode ? driveX : yUsed) // Drive forward with negative Y (forward)
                    .withVelocityY(slewMode ? driveY : xUsed) // Drive left with negative X (left)
                    .withRotationalRate(slewMode ? driveRotate : rotate); // Drive counterclockwise with negative X (left)
                
                // return drive.withVelocityX(slewMode ? driveX : -driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed) // Drive forward with negative Y (forward)
                //     .withVelocityY(slewMode ? driveY : (-driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed)) // Drive left with negative X (left)
                //     .withRotationalRate(slewMode ? driveRotate : -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate); // Drive counterclockwise with negative X (left)
            }
            )
        );
        
        // Idle
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Log
        drivetrain.registerTelemetry(logger::telemeterize);
        

        /* ========== DRIVER KEYBINDS ========== */
        // Normal

        // No SysID
        if (!enableSysIdKeybinds) {
            DRIVER.y().onTrue(new ResetIMU(drivetrain));
        }
        
        

        /* ========== OPERATOR KEYBINDS ========== */
        // Normal
        DRIVER.povDown()
            .onTrue(visionSub.setOperatorOverrideVisionTrustCommand(true))
            .onFalse(visionSub.setOperatorOverrideVisionTrustCommand(false));

        // No SysID
        if (!enableSysIdKeybinds) {
        }
        

        /* ========== TESTING KEYBINDS ========== */
        // SysID
        if (enableSysIdKeybinds) {
            DRIVER.back().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            DRIVER.back().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            DRIVER.start().and(DRIVER.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            DRIVER.start().and(DRIVER.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }
        // DRIVER.leftBumper().onTrue(new InstantCommand(() -> intakeSub.holdPosition()));
        // DRIVER.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.killIntakeWheels()));
        // DRIVER.leftBumper().onTrue(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.IDLE)).alongWith(
        //     new InstantCommand(() -> intakeSub.setDeployState(DeployState.STORE))
        // ));
        // DRIVER.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.INTAKE)).alongWith(
        //     new InstantCommand(() -> intakeSub.setDeployState(DeployState.DEPLOY))
        // ));
        DRIVER.leftTrigger().onTrue(coordSub.setIntakeCommand());

        // DRIVER.x().onTrue(new InstantCommand(() -> TurretSubsystem.getInstance().setManualTurretSetpoint(180)));
        // DRIVER.b().onTrue(new InstantCommand(() -> TurretSubsystem.getInstance().setManualTurretSetpoint(0)));
        DRIVER.x().onTrue(coordSub.decreaseManualAdjust());
        DRIVER.b().onTrue(coordSub.increaseManualAdjust());
        DRIVER.a().onTrue(coordSub.resetManualAdjust());
        DRIVER.leftStick().onTrue(coordSub.setManualTurret(false));
        DRIVER.leftStick().onFalse(coordSub.setManualTurret(true));
        DRIVER.povUp().onTrue(new InstantCommand(shooterSub::addManualHoodShift));
        DRIVER.povDown().onTrue(new InstantCommand(shooterSub::subtractManualHoodShift));
        // DRIVER.povUp().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().setManualHoodSetpoint(12)));
        // DRIVER.povDown().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().setManualHoodSetpoint(0)));
        DRIVER.povLeft().onTrue(new InstantCommand(shooterSub::resetManualHoodShift));
        
        DRIVER.povRight().onTrue(new InstantCommand(() -> intakeSub.setDeployState(DeployState.ZEROING)));
        DRIVER.rightBumper().onTrue(new InstantCommand(sidesweeperSub::setSlow).alongWith(
            new InstantCommand(transferSub::setSlowHopper)
        ));
        // DRIVER.leftTrigger().onFalse(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.IDLE)));

        // DRIVER.rightBumper().onTrue(new InstantCommand(() -> transferSub.setTransferState(TransferState.TRANSFER)));
        // DRIVER.rightBumper().onFalse(new InstantCommand(() -> transferSub.setTransferState(TransferState.IDLE)));

        // DRIVER.rightTrigger().onTrue(new InstantCommand(() -> sidesweeperSub.setSidesweeperState(SidesweeperState.TRANSFER)).alongWith(
        //     new InstantCommand(() -> sidesweeperSub.setRampState(RampState.TRANSFER)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.TRANSFER)))));
        // DRIVER.rightTrigger().onFalse(new InstantCommand(() -> sidesweeperSub.setSidesweeperState(SidesweeperState.IDLE)).alongWith(
        //     new InstantCommand(() -> sidesweeperSub.setRampState(RampState.IDLE)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.IDLE)))));
        DRIVER.rightTrigger().onTrue(coordSub.setFireCommand());
        DRIVER.rightBumper().onTrue(coordSub.setRejectingCommand(true));
        DRIVER.rightBumper().onFalse(coordSub.setRejectingCommand(false));
        
        // DRIVER.rightBumper().onTrue(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.EJECT)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.EJECT))));
        // DRIVER.rightBumper().onFalse(new InstantCommand(() -> sidesweeperSub.setSidesweeperState(SidesweeperState.IDLE)).alongWith(
        //     new InstantCommand(() -> sidesweeperSub.setRampState(RampState.IDLE)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.IDLE)))));

        // DRIVER.x().onTrue(new InstantCommand(() -> intakeSub.setPosition(17.6)));
        // DRIVER.rightTrigger().onTrue(new InstantCommand(() -> intakeSub.setPosition(1)));
    }

    private static double getRotation(double joyY, double joyX) {
        double rotation = Math.atan(joyY / joyX);

        if(joyY < 0 && joyX < 0) rotation = Math.PI - rotation;
        else if(joyY > 0 && joyX < 0) rotation = Math.PI - rotation;
        else if(joyY < 0 && joyX > 0) rotation *= -1;
        else if(joyY > 0 && joyX > 0) rotation *= -1;
        else if(joyY > 0) rotation = -Math.PI/2;
        else if(joyY < 0) rotation = Math.PI/2;
        else if(joyX < 0) rotation = Math.PI;
        else if (joyX > 0) rotation = 0;
        else rotation = 0;
        return rotation;
    }


    public ChassisSpeeds getRobotFieldCentricVeloc() {
        return new ChassisSpeeds(
            -driveBezier.getOutput(DRIVER.getLeftY()) * maxSpeed,
            -driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed,
            -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate
        );
    }

    private void configureNamedCommands() { // Set up the named commands (Using NamedCommands.registerCommand())
        NamedCommands.registerCommand("ZeroIntake", new InstantCommand());
        // NamedCommands.registerCommand("IntakePush", CoordinationSubsystem.getInstance().setIntakePushing());

        NamedCommands.registerCommand("SetIntake", coordSub.setIntakeCommand());
        NamedCommands.registerCommand("SetFire", coordSub.setFireCommand());
        // NamedCommands.registerCommand("SetFastTransfer", CoordinationSubsystem.getInstance().setFastTransferCommand());
        // NamedCommands.registerCommand("SetHardTrenchOverrideTrue", CoordinationSubsystem.getInstance().setHardTrenchOverride(true));
        // NamedCommands.registerCommand("SetHardTrenchOverrideFalse", CoordinationSubsystem.getInstance().setHardTrenchOverride(false));

        NamedCommands.registerCommand("SetFerryTargetBottom", new InstantCommand(() -> visionSub.setFerryTarget("bottom")));
        NamedCommands.registerCommand("SetFerryTargetTop", new InstantCommand(() -> visionSub.setFerryTarget("top")));

        NamedCommands.registerCommand("LowerShooterPower", new InstantCommand(shooterSub::subtractManualVelocityShift));
        NamedCommands.registerCommand("IncreaseShooterPower", new InstantCommand(() -> shooterSub.addManualVelocityShift(.25)));
        
        NamedCommands.registerCommand("LeftTurretAngle", new InstantCommand(turretSub::increaseManualAdjust));
        NamedCommands.registerCommand("RightTurretAngle", new InstantCommand(turretSub::decreaseManualAdjust));

        // NamedCommands.registerCommand("ClimberDown", new InstantCommand());

        NamedCommands.registerCommand("ForceTrench", coordSub.setForceTrenchCommand());
        NamedCommands.registerCommand("UseCustomTarget", visionSub.setUseCustomTarget());
    }

    private void configureAutos() { // Set up the auto chooser (Using drivetrain.getAutoPath())
        // autoChooser.addDefaultOption("Full Sweep", drivetrain.getAutoPath("P - Top Start", false));
        // autoChooser.addOption("Left Half Sweep + Depot", drivetrain.getAutoPath("HST - FPSD", false));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
