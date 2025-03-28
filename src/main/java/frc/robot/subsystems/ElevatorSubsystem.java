package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevationTarget;
import frc.robot.Constants.ScoringConstants.ScoringStates;
import edu.wpi.first.units.Units;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

  double targetPosition = 0;

  public static TalonFXConfiguration Elevator_Config = new TalonFXConfiguration();
  static {

    Elevator_Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    Elevator_Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorConstants.MAX_HEIGHT_INCHES; // Test
                                                                                                                   // Upper
                                                                                                                   // Limit
    Elevator_Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    Elevator_Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorConstants.MIN_HEIGHT_INCHES;

    Elevator_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Elevator_Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Elevator_Config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    Elevator_Config.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.ELEVATION_GEAR_RATIO;

    // TODO: Elevator Overshoot issue, tune kD and kV, kP
    Elevator_Config.Slot0.kG = 0.8; // 0.3
    Elevator_Config.Slot0.kS = 0.3; // 0.4
    Elevator_Config.Slot0.kV = 0.002; // 0.001
    Elevator_Config.Slot0.kA = 0.001; // 0.0
    Elevator_Config.Slot0.kP = 0.5; // 0.5
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
    

    rightMotorLeader.clearStickyFault_BootDuringEnable();
    leftMotorFollower.clearStickyFault_BootDuringEnable();
    topMotorFollower.clearStickyFault_BootDuringEnable();
    // TODO: Use a non-deprecated method
    // Invert the left motor
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    topMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    voltageRequest = new VoltageOut(0);
    motionRequest = new MotionMagicVoltage(0);
    resetSensorPosition(Constants.ElevatorConstants.MIN_HEIGHT_INCHES);
  }

  public double getElevatorPosition() {
    return rightMotorLeader.getPosition().getValueAsDouble();
  }

  public void setPosition(double height) {
    rightMotorLeader.setControl(motionRequest.withPosition(height));
    targetPosition = height;
  }

  public void stopPosition() {
    rightMotorLeader.stopMotor();
    leftMotorFollower.stopMotor();
    topMotorFollower.stopMotor();
  }

  public void resetSensorPosition(double height) {
    rightMotorLeader.setPosition(height);
    leftMotorFollower.setPosition(height);
    topMotorFollower.setPosition(height);
  }

  public void setElevatorState(ScoringStates state) {
    switch (state) {
      case Stow:
        setPosition(ElevationTarget.Stow.getValue());
        break;
    
      case Intake:
        setPosition(ElevationTarget.CoralIntake.getValue());
        break;
      case L1:
        setPosition(ElevationTarget.L1.getValue());
        break;
      case L2:
        setPosition(ElevationTarget.L2.getValue());
        break;
      case L3:
        setPosition(ElevationTarget.L3.getValue());
        break;
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Target", targetPosition);
    SmartDashboard.putNumber("Elevator Height", getElevatorPosition());
    SmartDashboard.putNumber("Motion Magic is Running", rightMotorLeader.getMotionMagicIsRunning().getValue().value);
  }
}