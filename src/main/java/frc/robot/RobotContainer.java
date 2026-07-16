// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
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

public class RobotContainer {
    public static final HopperTransferSubsystem transferSub = HopperTransferSubsystem.getInstance();
    public static final IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
    public static final ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
    public static final SidesweeperSubsystem sidesweeperSub = SidesweeperSubsystem.getInstance();
    public static final TurretSubsystem turretSub = TurretSubsystem.getInstance();
    public static final CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
    
    public static final CommandXboxController DRIVER = new CommandXboxController(RobotConstants.DRIVER_CONTROLLER_PORT);
    public static final CommandXboxController OPERATOR = new CommandXboxController(RobotConstants.OPERATOR_CONTROLLER_PORT);

    public static final boolean enableSysIdKeybinds = false; // Enable SysID keybinds, and disable conflicting keybinds! Currently disables: None
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("NewHopper/Auto Chooser");

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();
        configureAutos();
    }

    private void configureBindings() {
        /* ========== SWERVE KEYBINDS ========== */
        // Default
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DRIVER.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-DRIVER.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DRIVER.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        // Idle
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Logging
        drivetrain.registerTelemetry(logger::telemeterize);
        

        /* ========== DRIVER KEYBINDS ========== */
        DRIVER.back().onTrue(new ResetIMU(drivetrain));
        

        /* ========== OPERATOR KEYBINDS ========== */
        

        /* ========== TESTING KEYBINDS ========== */
        // SysID
        if (enableSysIdKeybinds) { // Enable SysID keybinds
            DRIVER.back().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            DRIVER.back().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            DRIVER.start().and(DRIVER.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            DRIVER.start().and(DRIVER.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        } else { // Disable conflicting keybinds
        }
    }

    private void configureNamedCommands() { // Set up the named commands (Using NamedCommands.registerCommand())
    }

    private void configureAutos() { // Set up the auto chooser (Using drivetrain.getAutoPath())
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
