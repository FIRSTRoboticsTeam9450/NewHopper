// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final Field2d field2d = new Field2d();
    private NetworkTable networkTable;
    private final SendableBuilderImpl sendableBuilder = new SendableBuilderImpl();

    private int updateValuesOffset = 0;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "NewHopper");
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        
        // Creates a network table which is read by Elastic (Seems like advantage scope doesn't see it)
        networkTable = NetworkTableInstance.getDefault()
                .getTable("FieldTable") // Name of the folder Field2d is stored in
                .getSubTable("Field2d"); // Name of the Field2d
        
        // Makes the builder attached to the network table to send the feild2d
        sendableBuilder.setTable(networkTable); 

        // initiate the field2d and builder
        field2d.initSendable(sendableBuilder);
        sendableBuilder.startListeners();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        field2d.setRobotPose(RobotContainer.drivetrain.getState().Pose);
        sendableBuilder.update();
    }

    @Override
    public void disabledInit() {
        VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().disabledStdDev);
        VisionSubsystem.getInstance().setThrottle(VisionConstants.kDisabledThrottle);
        VisionSubsystem.getInstance().setLLIMUModes(VisionConstants.kDisabledIMUMode);
        updateValuesOffset = 0;
    }

    @Override
    public void disabledPeriodic() {
        if (updateValuesOffset == 15) IntakeSubsystem.getInstance().updateConstants();
        else if (updateValuesOffset == 30) ShooterSubsystem.getInstance().updateShooterConstants();
        else if (updateValuesOffset >= 45) {
            TurretSubsystem.getInstance().updateTunableParameters();
            updateValuesOffset = 0;
        }
        updateValuesOffset++;
    }

    @Override
    public void disabledExit() {
        VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().enabledMT2StdDev);
        VisionSubsystem.getInstance().setThrottle(VisionConstants.kEnabledThrottle);
        VisionSubsystem.getInstance().setLLIMUModes(VisionConstants.kEnabledIMUMode);
        // VisionSubsystem.getInstance().setLLIMUAssistAlpha(0.01);
    }

    @Override
    public void autonomousInit() {
        VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().enabledMT2StdDev);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
