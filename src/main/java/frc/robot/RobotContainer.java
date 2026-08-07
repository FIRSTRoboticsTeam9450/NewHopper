// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
import frc.robot.subsystems.HopperTransferSubsystem.TransferState;
import frc.robot.subsystems.IntakeSubsystem.DeployState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.SidesweeperSubsystem.RampState;
import frc.robot.subsystems.SidesweeperSubsystem.SidesweeperState;
import frc.robot.util.BezierCurve;

public class RobotContainer {
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final VisionSubsystem visionSub = VisionSubsystem.getInstance();
    public static final HopperTransferSubsystem transferSub = HopperTransferSubsystem.getInstance();
    public static final IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
    public static final ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
    public static final SidesweeperSubsystem sidesweeperSub = SidesweeperSubsystem.getInstance();
    public static final TurretSubsystem turretSub = TurretSubsystem.getInstance();
    public static final CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
    
    public static final CommandXboxController DRIVER = new CommandXboxController(RobotConstants.DRIVER_CONTROLLER_PORT);
    public static final CommandXboxController OPERATOR = new CommandXboxController(RobotConstants.OPERATOR_CONTROLLER_PORT);

    public static final boolean enableSysIdKeybinds = false; // Enable SysID keybinds, and disable conflicting keybinds!
        // Currently disables:
        //  Driver:
        //   Reset IMU
        //  Operator:
    
    /**
     * Woojins Drive: 117.4, 0.054, 91.4, 0.76, 0.1, 0.01
     * Woojins Rotate: 120.1, 0.145, 92.2, 0.362, 0.035, 0.03
     */
    public static BezierCurve driveBezier = new BezierCurve("drive", 151.0, 0.137, 116, 0.646, 0.1, 0.01);
    public static BezierCurve rotateBezier = new BezierCurve("rotate", 164.8, 0.093, 100, 0.84, 0.035, 0.03);
    
    public static SlewRateLimiter driveXAccelLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter driveYAccelLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter rotationAccelLimiter = new SlewRateLimiter(8.0);
    
    private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static boolean slewMode = false;

    public static double maxSpeed = DEFAULT_MAX_SPEED * 0.8;
    public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE * 1;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(driveBezier.getDeadband())
            .withRotationalDeadband(rotateBezier.getDeadband())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("NewHopper/Auto Chooser");

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();
        configureAutos();
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
                double rotation = Math.atan(joyY / joyX);
                if(joyY < 0 && joyX < 0) rotation = Math.PI - rotation;
                else if(joyY > 0 && joyX < 0) rotation = Math.PI - rotation;
                else if(joyY < 0 && joyX > 0) rotation *= -1;
                else if(joyY > 0 && joyX > 0) rotation *= -1;
                else if(joyY > 0 && joyX == 0) rotation = -Math.PI/2;
                else if(joyY < 0 && joyX == 0) rotation = Math.PI/2;
                else if(joyY == 0 && joyX < 0) rotation = Math.PI;
                else if (joyY == 0 && joyX > 0) rotation = 0;
                else if(joyY == 0 && joyX > 0) rotation = 0;
                else if(joyY == 0 && joyX == 0) rotation = 0;
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
            .onTrue(VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(true))
            .onFalse(VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(false));

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
        DRIVER.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.IDLE)).alongWith(
            new InstantCommand(() -> intakeSub.setDeployState(DeployState.STORE))
        ));
        DRIVER.leftTrigger().onFalse(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.INTAKE)).alongWith(
            new InstantCommand(() -> intakeSub.setDeployState(DeployState.DEPLOY))
        ));
        DRIVER.x().onTrue(CoordinationSubsystem.getInstance().decreaseManualAdjust());
        DRIVER.b().onTrue(CoordinationSubsystem.getInstance().increaseManualAdjust());
        DRIVER.a().onTrue(CoordinationSubsystem.getInstance().resetManualAdjust());
        DRIVER.leftStick().onTrue(CoordinationSubsystem.getInstance().setManualTurret(false));
        DRIVER.leftStick().onFalse(CoordinationSubsystem.getInstance().setManualTurret(true));
        DRIVER.povUp().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().addManualHoodShift()));
        DRIVER.povDown().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualHoodShift()));
        DRIVER.povLeft().onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().resetManualHoodShift()));
        
        DRIVER.povRight().onTrue(new InstantCommand(() -> intakeSub.setDeployState(DeployState.ZEROING)));
        // DRIVER.leftTrigger().onFalse(new InstantCommand(() -> intakeSub.setIntakeState(IntakeState.IDLE)));

        // DRIVER.rightBumper().onTrue(new InstantCommand(() -> transferSub.setTransferState(TransferState.TRANSFER)));
        // DRIVER.rightBumper().onFalse(new InstantCommand(() -> transferSub.setTransferState(TransferState.IDLE)));

        DRIVER.rightTrigger().onTrue(new InstantCommand(() -> sidesweeperSub.setSidesweeperState(SidesweeperState.TRANSFER)).alongWith(
            new InstantCommand(() -> sidesweeperSub.setRampState(RampState.TRANSFER)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.TRANSFER)))));
        DRIVER.rightTrigger().onFalse(new InstantCommand(() -> sidesweeperSub.setSidesweeperState(SidesweeperState.IDLE)).alongWith(
            new InstantCommand(() -> sidesweeperSub.setRampState(RampState.IDLE)).alongWith(new InstantCommand(() -> transferSub.setTransferState(TransferState.IDLE)))));

        // DRIVER.x().onTrue(new InstantCommand(() -> intakeSub.setPosition(17.6)));
        // DRIVER.rightTrigger().onTrue(new InstantCommand(() -> intakeSub.setPosition(1)));
    }

    
    public ChassisSpeeds getRobotFieldCentricVeloc() {
        return new ChassisSpeeds(
            -driveBezier.getOutput(DRIVER.getLeftY()) * maxSpeed,
            -driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed,
            -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate
        );
    }

    private void configureNamedCommands() { // Set up the named commands (Using NamedCommands.registerCommand())
    }

    private void configureAutos() { // Set up the auto chooser (Using drivetrain.getAutoPath())
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
