package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.elevator.HomeToElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX bottomLeft = new TalonFX(0, "Elevator");
  public TalonFX bottomRight = new TalonFX(1, "Elevator");
  public TalonFX topMotor = new TalonFX(2, "Elevator");

  public double position = 0;
  public int idxLevel = 0;

  public ElevatorSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.ElevatorConstants.kP;

    bottomLeft.getConfigurator().apply(config);
    bottomLeft.getConfigurator().apply(slot0Configs);
    setGoal(1);
    this.setDefaultCommand(new HomeToElevatorPosition(this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder pos", bottomLeft.getRotorPosition().getValueAsDouble());
  }
   public void setGoal(double elevatorOffset) {
        position = Constants.ElevatorConstants.elevatorBottomLimit + elevatorOffset * (Constants.ElevatorConstants.elevatorTopLimit - Constants.ElevatorConstants.elevatorBottomLimit);
        bottomLeft.set(position);
        bottomRight.set(position);
    }
    public void setGoalAsTickOffset(double elevatorOffsetTicks) {
        position = elevatorOffsetTicks;
        bottomLeft.set(position);
        bottomRight.set(position);
    }

    public boolean isAtTarget() {
        // var rotorPos = bottomLeft.getRotorPosition().getValueAsDouble();
        double err = Math.abs(bottomLeft.getRotorPosition().getValueAsDouble() - position);
        double errRight = Math.abs(bottomRight.getRotorPosition().getValueAsDouble()- position);
        return err < 0.1 && errRight < 0.1;
    }
    public boolean isAtTarget(double setpoint) {
        double err = Math.abs(bottomLeft.getRotorPosition().getValueAsDouble() - setpoint);
        double errRight = Math.abs(bottomRight.getRotorPosition().getValueAsDouble() - setpoint);
        return err < 0.1 && errRight < 0.1;
    }

  
}