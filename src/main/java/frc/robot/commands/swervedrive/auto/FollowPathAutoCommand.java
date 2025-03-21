package frc.robot.commands;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

// YAGSL drive system instea of swerve drive
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FollowAuto extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Command autoCommand;

  public FollowPathAuto(SwerveDrive swerveDrive) {
    this.swerveSubsystem = swerveSubsystem;
    autoCommand = new PathPlannerAuto("center_preload_blue_auto");
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    autoCommand.initialize();
  }

  @Override
  public void execute() {
    autoCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    autoCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return autoCommand.isFinished();
  }
}
