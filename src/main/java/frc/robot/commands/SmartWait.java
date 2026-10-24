package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SmartWait extends Command{

    private Timer timer = new Timer();
    private double minEndTime = 0;
    private double maxEndTime = 0;
    private double endTime = 0;
    public SmartWait(double minEnd, double maxEnd) {
        minEndTime = minEnd;
        maxEndTime = maxEnd;
    }

    @Override
    public void initialize() {
        timer.restart();
        endTime = minEndTime;
        
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(endTime);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

}
