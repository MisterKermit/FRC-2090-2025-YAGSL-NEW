package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ScoringConstants.ScoringStates;
import frc.robot.Constants.WristConstants.WristStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;

public class ScoringMacro extends Command {
    // private final ArmSubsystem arm;
    // private final ElevatorSubsystem elevator;
    // private final WristSubsystem wrist;

    private final ScoringSubsystem scoring;

    private ScoringStates state;
    
    public ScoringMacro(ScoringSubsystem scoring, ScoringStates state) {
        this.scoring = scoring;
        this.state = state;
        addRequirements(scoring);
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        switch (state) {
            case Stow:
                scoring.setArmPivotState(state);
                scoring.setWristState(state);
                // .setElevatorState(state);
                break;
        
            case Intake:
                scoring.setArmPivotState(state);
                // elevator.elevateCommandState(ElevationTarget.L1);
                scoring.setArmPivotState(state);
                break;
        }
    }

    @Override
    public void end(boolean isFinished) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
