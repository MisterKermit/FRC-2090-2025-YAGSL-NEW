package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMath.Arm;

public class ArmSubsystem extends SubsystemBase{
    
    private final SparkMax arm; 
    private final SparkMax wrist;
    
    // private final SparkClosedLoopController arm_c = arm.getClosedLoopController();
    // private final SparkClosedLoopController wrist_c = wrist.getClosedLoopController();

    private RelativeEncoder arm_rencoder;
    private RelativeEncoder wrist_rencoder;

    private AbsoluteEncoder arm_aencoder;
    private AbsoluteEncoder wrist_aencoder;

    private final SparkClosedLoopController pidController;
    private final ArmFeedforward        arm_Feedforward = new ArmFeedforward(0, 0, 0);

    private final double targetPos = 0;
  
    public ArmSubsystem() {
      arm = new SparkMax(Constants.ArmConstants.arm_left_id, SparkLowLevel.MotorType.kBrushless); 
      wrist = new SparkMax(Constants.ArmConstants.arm_right_id, SparkLowLevel.MotorType.kBrushless);
      SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);

        config
            .encoder
            .countsPerRevolution(4096)
            .positionConversionFactor(1.0);

        config
            .closedLoop
            .p(1)
            .i(0)
            .d(0)
            .iMaxAccum(0.1)
            .outputRange(-0.2, 0.2);
        
        arm.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        this.arm_rencoder = arm.getEncoder();
        this.wrist_rencoder = wrist.getEncoder();

        pidController = arm.getClosedLoopController();
        
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Current Arm pos", arm_rencoder.getPosition());
    }

    public boolean isAlgaeArmAtPosition(double position) {
      position = MathUtil.clamp(position, 0, 1.0); // Assuming the range is between 0 and 1

      return Math.abs(arm_aencoder.getPosition() - position) < 0.05;
    }

    public double getAlgaeArmPosition() {
      return arm_aencoder.getPosition();
    }

    public boolean isAlgaeArmAtPosition() {
      return isAlgaeArmAtPosition(targetPos);
    }

    
    public Command moveArmToPosition(double position) {
      return runEnd(() -> setAlgaeArmPosition(position), this::stopAlgaeArm)
          .until(this::isAlgaeArmAtPosition);
    }

    public Command stop() {
      return runOnce(this::stopAlgaeArm);
    }

    private void stopAlgaeArm() {
      arm.stopMotor();
    }

    public void setAlgaeArmPosition(double position) {
      pidController.setReference(position, ControlType.kPosition);
    }

    public Command set(double position) {
      return runEnd(() -> setAlgaeArmPosition(position), this::stopAlgaeArm)
          .until(this::isAlgaeArmAtPosition);
    }

    public Command setArmToIntake() {
      return set(0.5);
    }
}
