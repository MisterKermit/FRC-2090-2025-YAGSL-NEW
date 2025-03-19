package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ScoringConstants.ScoringStates;
import frc.robot.Constants.WristConstants.WristStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;

public class ScoringMacro extends Command {
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;

    private ScoringStates state;
    
    public ScoringMacro(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist, ScoringStates state) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.state = state;
        addRequirements(arm, elevator, wrist);
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        switch (state) {
            case Stow:
                arm.setArmStatePivot(ArmStates.Stow);
                elevator.elevateCommandState(ElevationTarget.CoralIntake);
                wrist.setWristStatePivot(WristStates.Stow);
                break;
        
            case Intake:
                arm.setArmStatePivot(ArmStates.Intake);
                elevator.elevateCommandState(ElevationTarget.L1);
                wrist.setWristStatePivot(WristStates.Intake);
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
