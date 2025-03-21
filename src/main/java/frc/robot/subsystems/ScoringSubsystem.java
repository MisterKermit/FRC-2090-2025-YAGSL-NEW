
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.WristConstants;
// import frc.robot.Constants.Wrist;
// import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.Constants.ScoringConstants.ScoringStates;

public class ScoringSubsystem extends SubsystemBase {
    private SparkMaxConfig armPivotConfig = new SparkMaxConfig();
    private SparkMaxConfig wristConfig = new SparkMaxConfig();

    // initialize arm pivot
    private SparkMax armPivot = new SparkMax(ArmConstants.arm_id, MotorType.kBrushless);
    private SparkClosedLoopController armPivotController = armPivot.getClosedLoopController();
    private RelativeEncoder armPivotEncoder = armPivot.getEncoder();
    public static double kArmP = 0.5;
    public static double kArmI = 0;
    public static double kArmD = 0;
    public static double armAngleSetPoint = 0;

    // initialize intake pivot
    private SparkMax wrist = new SparkMax(WristConstants.wrist_id, MotorType.kBrushless);
    private SparkClosedLoopController wristController = wrist.getClosedLoopController();
    private RelativeEncoder wristEncoder = wrist.getEncoder();
    public static double kWristP = 0.1;
    public static double kWristI = 0;
    public static double kWristD = 0.01;
    public static double wristAngleSetPoint = 0;

    public static ScoringStates currentState;

    public ScoringSubsystem() {
        // create configs
        armPivotConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(  80)
                .voltageCompensation(12);
        armPivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kArmP, kArmI, kArmD)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(300000)
                .maxAcceleration( 60000)
                .allowedClosedLoopError(1)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        armPivotConfig.encoder
                // degrees
                .positionConversionFactor(Constants.ArmConstants.rotations_to_degrees)
                // degrees per sec
                .velocityConversionFactor(1);

        wristConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(80)
                .voltageCompensation(12);
        wristConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kWristP, kWristI, kWristD)
                .outputRange(-1, 1).maxMotion
                .maxVelocity(11000)
                .maxAcceleration(4000)
                .allowedClosedLoopError(1)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        wristConfig.encoder
                // degrees
                .positionConversionFactor(Constants.WristConstants.rotations_to_degrees)
                // degrees per sec
                .velocityConversionFactor(360 / 60);


        // armPivotEncoder.setPosition(0);
        // wristEncoder.setPosition(0);
        // set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public Trigger atArmAngle(double angle, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(angle, armPivotEncoder.getPosition(), tolerance));
    }

    public Trigger atWristAngle(double angle, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(angle, wristEncoder.getPosition(), tolerance));
    }

    public void reachArmPivotTarget(double target) {
        armAngleSetPoint = target;
        armPivotController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setArmPivotTarget(double target) {
        armAngleSetPoint = target;
        return run(() -> reachArmPivotTarget(target));
    }

    public Command setManualArm(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            wristAngleSetPoint += rightJoystick.getAsDouble();
            reachWristTarget(wristAngleSetPoint);
            if (wristAngleSetPoint < 59 && armAngleSetPoint > 45) {
                armAngleSetPoint = 45;
            }
            armAngleSetPoint += leftJoystick.getAsDouble();
            reachArmPivotTarget(armAngleSetPoint);
        });
    }

    public Command stopWholeArm() {
        return run(() -> {
            armPivot.set(0);
            wrist.set(0);
        });
    }

    public Command setManualArmVoltageWithLimiter(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            double armSpeed = leftJoystick.getAsDouble() * 0.6;
            double wristSpeed = rightJoystick.getAsDouble() * 0.6;
            if (armAngleSetPoint <= 45 || wristAngleSetPoint >= 59) {
                wrist.set(wristSpeed);
                armPivot.set(armSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
                armAngleSetPoint = armPivotEncoder.getPosition();
            } else if (wristAngleSetPoint < 59 && armAngleSetPoint > 45 && armSpeed <= 0) {
                wrist.set(wristSpeed);
                armPivot.set(armSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
                armAngleSetPoint = armPivotEncoder.getPosition();
            } else if (wristAngleSetPoint < 59 && armAngleSetPoint > 45 && armSpeed >= 0) {
                wrist.set(wristSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
            }
        });
    }

    public Command setManualArmVoltage(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            double armSpeed = leftJoystick.getAsDouble() * 0.6;
            double wristSpeed = rightJoystick.getAsDouble() * 0.6;
            wrist.set(wristSpeed);
            armPivot.set(armSpeed);
            wristAngleSetPoint = wristEncoder.getPosition();
            armAngleSetPoint = armPivotEncoder.getPosition();
        });
    }

    public void setArmPivotState(ScoringStates state) {
        switch (state) {
            case Stow:
                reachArmPivotTarget(Constants.ArmConstants.stow_angle);
                break;

            case Intake:
                reachArmPivotTarget(Constants.ArmConstants.intake_angle);
                break;
        }
        currentState = state;
    }

    public Command setArmPivotStateCommand(ScoringStates state) {
        return runOnce(() -> {
            switch (state) {
                case Stow:
                    reachArmPivotTarget(Constants.ArmConstants.stow_angle);
                    break;

                case Intake:
                    reachArmPivotTarget(ArmConstants.intake_angle);
                    break;
            }
        });
    }

    public void setWristState(ScoringStates state) {
        switch (state) {
            case Stow:
                reachWristTarget(WristConstants.stow_angle);
                break;

            case Intake:
                reachWristTarget(WristConstants.intake_angle);
                break;
        }
        currentState = state;
    }


    public ScoringStates returnState() {
        return currentState;
    }

    public void reachWristTarget(double target) {
        wristAngleSetPoint = target;
        wristController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setWristTarget(double target) {
        wristAngleSetPoint = target;
        return run(() -> reachWristTarget(target));
    }

    public void resetArmPivotEncoder() {
        armPivotEncoder.setPosition(0);
    }

    public void resetWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public double getArmPivotEncoderPos() {
        return armPivotEncoder.getPosition();
    }

    public double getWristEncoderPos() {
        return wristEncoder.getPosition();
    }

    public Command wristForward() {
        return run(() -> wrist.set(0.1));
    }

    public Command wristBackwards() {
        return run(() -> wrist.set(-0.5));
    }

    public Command stopWrist() {
        return run(() -> wrist.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armPivotEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("ArmAngleSetpoint", armAngleSetPoint);
        SmartDashboard.putNumber("WristAngleSetpoint", wristAngleSetPoint);

        // if (Constants.Testing.testingArm) {
        // reachArmPivotTarget(SmartDashboard.getNumber("ArmAngleSetpoint", 0));
        // }
        // if (Constants.Testing.testingWrist) {
        // reachWristTarget(SmartDashboard.getNumber("WristAngleSetpoint", 0));
        // }
    }
}
