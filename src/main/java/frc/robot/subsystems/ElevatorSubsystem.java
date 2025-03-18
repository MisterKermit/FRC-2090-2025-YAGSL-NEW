package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.units.Units;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ElevatorSubsystem extends SubsystemBase {
  
  // We have two Falcon 500s
  // TODO: Specify CAN IDss
  private static final TalonFX leftMotorFollower = new TalonFX(13, "Elevator");
  private static final TalonFX rightMotorLeader = new TalonFX(12, "Elevator");
  private static final TalonFX topMotorFollower = new TalonFX(11, "Elevator");

  MotionMagicVoltage motionRequest;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest = new VoltageOut(0);

  double currentLeftPosition = 0;
  double currentRightPosition = 0;
  double targetPosition = 0;

  public static TalonFXConfiguration Elevator_Config = new TalonFXConfiguration();
  static{

    Elevator_Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    Elevator_Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorConstants.MAX_HEIGHT_INCHES; // Test Upper Limit
    Elevator_Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    Elevator_Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorConstants.MIN_HEIGHT_INCHES;
    
    Elevator_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Elevator_Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    Elevator_Config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    Elevator_Config.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.ELEVATION_GEAR_RATIO;

    //TODO: Elevator Overshoot issue, tune kD and kV, kP
    Elevator_Config.Slot0.kG = 0.6; //0.3 
    Elevator_Config.Slot0.kS = 0.05; //0.4
    Elevator_Config.Slot0.kV = 0.02; //0.001
    Elevator_Config.Slot0.kA = 0.001; //0.0
    Elevator_Config.Slot0.kP = 0.5; //0.5
    Elevator_Config.Slot0.kI = 0.0;
    Elevator_Config.Slot0.kD = 0.0;
    Elevator_Config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    Elevator_Config.MotionMagic.MotionMagicCruiseVelocity = 600;
    Elevator_Config.MotionMagic.MotionMagicAcceleration = 300;
    Elevator_Config.MotionMagic.MotionMagicExpo_kV = 0.12;

    Elevator_Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    Elevator_Config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    Elevator_Config.CurrentLimits.SupplyCurrentLimit = 60;
    Elevator_Config.CurrentLimits.SupplyCurrentLowerTime = 1;

  }

  public ElevatorSubsystem() {
    rightMotorLeader.getConfigurator().apply(Elevator_Config);
    leftMotorFollower.getConfigurator().apply(Elevator_Config);
    topMotorFollower.getConfigurator().apply(Elevator_Config);
    // TODO: Use a non-deprecated method
    // Invert the left motor
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    topMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    voltageRequest = new VoltageOut(0);
    motionRequest = new MotionMagicVoltage(0);
    resetSensorPosition(Constants.ElevatorConstants.MIN_HEIGHT_INCHES);
  }

  public double getElevatorPosition(){
    return rightMotorLeader.getPosition().getValueAsDouble();
  }

  public void setPosition(double height) {
    rightMotorLeader.setControl(motionRequest.withPosition(height));
    targetPosition = height;
  }

  public void resetSensorPosition(double height) {
    rightMotorLeader.setPosition(height);
    leftMotorFollower.setPosition(height);
    topMotorFollower.setPosition(height);
  }

  public Command elevateCommand(double target) {
    return new ElevateCommand(this, target);
  }

  public Command elevateCommand(ElevationTarget target) {
    return elevateCommand(target.getValue());
  }
  
  private class ElevateCommand extends Command {
    private double elevateTarget;

    private static final double ELEVATE_COMMAND_DEADBAND = 0.25; //inches

    public ElevateCommand(ElevatorSubsystem elevator, double elevateTarget) {
      elevateTarget = Math.max(Math.min(elevateTarget, Constants.ElevatorConstants.MAX_HEIGHT_INCHES), Constants.ElevatorConstants.MIN_HEIGHT_INCHES);
      this.elevateTarget = elevateTarget;
      addRequirements(elevator);
      // It's fine to ignore the joystick because the profile should not take very long
    }

    @Override // every 20ms
    public void execute() {
      SmartDashboard.putNumber("Elevator Target", elevateTarget);
      setPosition(elevateTarget);

    }
    @Override
    public void end(boolean isInterrupted) {
      setPosition(getElevatorPosition());
    }

    @Override
    public boolean isFinished() {
      return Math.abs(getElevatorPosition() - elevateTarget) < ELEVATE_COMMAND_DEADBAND;
    }
  }

  public enum ElevationTarget {
    // https://www.desmos.com/calculator/ocl2iqiu7n
    // Unit: inches
    CoralIntake(3),
    L1(13.899),
    L2(24.899),
    L3(40.298),
    AlgaeL2(20.899),
    AlgaeL3(36.899)
    ;

    private double targetValue;
    private ElevationTarget(double targetValue) {
      this.targetValue = targetValue;
    }
    public double getValue() {
      return targetValue;
    }
  }

  public void elevateAtVoltage(double voltage) {
    rightMotorLeader.setVoltage(voltage);
  }
  

  private class ManualElevationCommand extends Command {
    private final CommandXboxController controller;


    public ManualElevationCommand(ElevatorSubsystem elevator, CommandXboxController controller) {
      this.controller = controller;
      addRequirements(elevator);
    }


    @Override
    public void initialize() {
    }

    @Override // every 20ms
    public void execute() {
      double input = -controller.getLeftY();
      // if (Math.abs(input) > Constants.ElevatorConstants.JOYSTICK_DEADBAND) {
      //   targetPosition = targetPosition + input;
      // }
      // targetPosition = Math.max(Math.min(targetPosition, Constants.ElevatorConstants.MAX_HEIGHT_INCHES), Constants.ElevatorConstants.MIN_HEIGHT_INCHES);
      // setPosition(targetPosition);

      // Only update the target position if input is outside the deadband
      if (Math.abs(input) > Constants.ElevatorConstants.JOYSTICK_DEADBAND) {
        targetPosition += input * 0.5; // Scale input to avoid excessive movement
      }

      // Clamp the target position to within the valid elevator range
      targetPosition = Math.max(Math.min(targetPosition, Constants.ElevatorConstants.MAX_HEIGHT_INCHES), Constants.ElevatorConstants.MIN_HEIGHT_INCHES);

      // Continuously set the elevator to the last stored target position
      setPosition(targetPosition);
    }
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return new ManualElevationCommand(this, controller);
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Target", targetPosition);
    SmartDashboard.putNumber("Elevator Height", getElevatorPosition());
    SmartDashboard.putNumber("Motion Magic is Running", rightMotorLeader.getMotionMagicIsRunning().getValue().value );
  }

}