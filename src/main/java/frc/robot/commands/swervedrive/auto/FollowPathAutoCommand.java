// package frc.robot.commands;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.SwerveDrive;

// public class FollowPathAuto extends CommandBase {
//   private final SwerveDrive swerveDrive;
//   private final PPSwerveControllerCommand swerveCommand;

//   public FollowPathAuto(SwerveDrive swerveDrive) {
//     this.swerveDrive = swerveDrive;
//     // load "SamplePath" from pathPlanner
//     // default maximum velocity and acceleration --> 3.0 m/s and 3.0 m/s2, change as needed
//     PathPlannerTrajectory trajectory = PathPlanner.loadPath("SamplePath", 3.0, 3.0);

//     // PPSwerveControllerCommand (command class in Pathplanner library) that will follow the trajectory 
//     swerveCommand = new PPSwerveControllerCommand(
//          trajectory,
//         swerveDrive::getPose,             // robot pose
//         swerveDrive.getKinematics(),      // swerve drive kinematics
//         swerveDrive.getXController(),     // PID controller for X 
//         swerveDrive.getYController(),     // PID controller for Y 
//         swerveDrive.getThetaController(), // PID controller for rotation
//         swerveDrive::setModuleStates,     // takes trajectory or path-following controller ex. wheel speeds and angles, and applies to swerve drive system
//         swerveDrive                       // drive subsystem
//     );
//     // ensure above code is correct, I made it look nice :)
//     addRequirements(swerveDrive);
//   }

//   @Override
//   public void initialize() {
//     swerveCommand.initialize();
//   }

//   @Override
//   public void execute() {
//     swerveCommand.execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerveCommand.end(interrupted);
//   }

//   @Override
//   public boolean isFinished() {
//     return swerveCommand.isFinished();
//   }
// }
