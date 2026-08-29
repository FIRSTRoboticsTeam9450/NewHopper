package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetIMU extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private boolean finished = false;

  public ResetIMU(CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    drivetrain.seedFieldCentric();
    finished = true;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
