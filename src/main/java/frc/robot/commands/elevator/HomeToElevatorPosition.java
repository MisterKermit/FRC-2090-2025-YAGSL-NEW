package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeToElevatorPosition extends Command {
    ElevatorSubsystem elevatorSubsystem;

    public HomeToElevatorPosition(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {

        if(!RobotContainer.driverXbox.rightBumper().getAsBoolean()){
            elevatorSubsystem.setGoalAsTickOffset(Constants.ElevatorConstants.elevatorLevels[this.elevatorSubsystem.idxLevel]);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}