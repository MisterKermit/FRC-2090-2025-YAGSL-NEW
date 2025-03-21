package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToAprilTag extends Command {

    private final SwerveSubsystem driveTrain;

    private final VisionSubsystem limelightSubsystem;

    public DriveToAprilTag(SwerveSubsystem swerb, VisionSubsystem limelight) {
        this.driveTrain = swerb;
        this.limelightSubsystem = limelight;
        addRequirements(swerb, limelight);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}