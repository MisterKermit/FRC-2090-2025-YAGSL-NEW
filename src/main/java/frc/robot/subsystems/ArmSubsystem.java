package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.fasterxml.jackson.databind.ser.BeanSerializer;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {

  private SparkMax arm = new SparkMax(Constants.ArmConstants.arm_id, SparkLowLevel.MotorType.kBrushless);

  private SparkMaxConfig config = new SparkMaxConfig();

  private RelativeEncoder arm_encoder = arm.getEncoder();

  private final SparkClosedLoopController pidController = arm.getClosedLoopController();

  private static double armsetPoint = 0;

  public ArmSubsystem() {
    config
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(40)
        .voltageCompensation(12);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1)
        .pidf(0.1, 0, 0, 0).maxMotion
        .maxVelocity(10)
        .maxAcceleration(5)
        .allowedClosedLoopError(1)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    config.encoder
        .positionConversionFactor(Constants.ArmConstants.rotations_to_degrees)
        .velocityConversionFactor(Constants.ArmConstants.velocity_conversion);

    arm_encoder.setPosition(0);

    arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Relative Enc pos", getAlgaeArmPosition());
    // SmartDashboard.putNumber("Arm Target Pos", armsetPoint);
    // SmartDashboard.putNumber("Arm Velocity", arm_encoder.getVelocity());
  }

  public double getAlgaeArmPosition() {
    return arm_encoder.getPosition();
  }

  public void setAlgaeArmPosition(double position) {
    armsetPoint = position;
    pidController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  public Command rotateArm(double target) {
    return run(() -> setAlgaeArmPosition(target));
  }

  public void setArmStatePivot(Constants.ArmConstants.ArmStates state) {
    switch (state) {
      case Stow:
        armsetPoint = Constants.ArmConstants.stow_angle;
        break;
      case Intake:
        armsetPoint = Constants.ArmConstants.intake_angle;
        break;
    }
    setAlgaeArmPosition(armsetPoint);
  }

  public Command setStateArm(ArmStates states) {
    return run(() -> setArmStatePivot(states));
  }

  // Temp testing commnands
  public Command runArmUp() {
    return run(() -> arm.set(1));
  }

  public Command runArmDown() {
    return run(() -> arm.set(-10));
  }

}
