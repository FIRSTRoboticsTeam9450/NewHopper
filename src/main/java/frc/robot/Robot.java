// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionConstants;

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
        //RobotContainer.testLog.run();
    }

    @Override
    public void disabledInit() {
        RobotContainer.visionSub.setVisionMeasurementStdDevs(RobotContainer.visionSub.disabledStdDev);
        RobotContainer.visionSub.setThrottle(VisionConstants.kDisabledThrottle);
        RobotContainer.visionSub.setLLIMUModes(VisionConstants.kDisabledIMUMode);
        updateValuesOffset = 0;
    }

    @Override
    public void disabledPeriodic() {
        if (updateValuesOffset == 15) RobotContainer.intakeSub.updateConstants();
        else if (updateValuesOffset == 30) RobotContainer.shooterSub.updateShooterConstants();
        else if (updateValuesOffset >= 45) {
            RobotContainer.turretSub.updateTunableParameters();
            updateValuesOffset = 0;
        }
        updateValuesOffset++;
    }

    @Override
    public void disabledExit() {
        RobotContainer.visionSub.setVisionMeasurementStdDevs(RobotContainer.visionSub.enabledMT2StdDev);
        RobotContainer.visionSub.setThrottle(VisionConstants.kEnabledThrottle);
        RobotContainer.visionSub.setLLIMUModes(VisionConstants.kEnabledIMUMode);
        // VisionSubsystem.getInstance().setLLIMUAssistAlpha(0.01);
    }

    @Override
    public void autonomousInit() {
        RobotContainer.visionSub.setVisionMeasurementStdDevs(RobotContainer.visionSub.enabledMT2StdDev);
        RobotContainer.coordSub.setIsAuto(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        RobotContainer.coordSub.setIsAuto(false);
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
        RobotContainer.coordSub.setIsAuto(false);
        
        SignalLogger.start();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

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
