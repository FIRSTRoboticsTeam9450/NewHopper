// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.HopperTransferSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SidesweeperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    public static final HopperTransferSubsystem transferSub = HopperTransferSubsystem.getInstance();
    public static final IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
    public static final ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
    public static final SidesweeperSubsystem sidesweeperSub = SidesweeperSubsystem.getInstance();
    public static final TurretSubsystem turretSub = TurretSubsystem.getInstance();

    public static final CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
    
    public static final CommandXboxController DRIVER = new CommandXboxController(RobotConstants.DRIVER_CONTROLLER_PORT);
    public static final CommandXboxController OPERATOR = new CommandXboxController(RobotConstants.OPERATOR_CONTROLLER_PORT);
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DRIVER.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-DRIVER.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DRIVER.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // DRIVER.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // DRIVER.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-DRIVER.getLeftY(), -DRIVER.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // DRIVER.back().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // DRIVER.back().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // DRIVER.start().and(DRIVER.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // DRIVER.start().and(DRIVER.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on button press.
        DRIVER.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // Reset IMU

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
