package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {

    private SparkMax wrist = new SparkMax(Constants.WristConstants.wrist_id, MotorType.kBrushless);

    private SparkMaxConfig wrist_config = new SparkMaxConfig();

    private final RelativeEncoder wrist_encoder = wrist.getEncoder();

    private final SparkClosedLoopController wrist_controller = wrist.getClosedLoopController();

    private double wrist_target = 0;

    public WristSubsystem() {
        wrist_config
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .smartCurrentLimit(80)
                .voltageCompensation(12);
        wrist_config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1)
                .pid(0.1, 0, 0).maxMotion
                .maxVelocity(10)
                .maxAcceleration(5)
                .allowedClosedLoopError(5);
        wrist_config.encoder
                .positionConversionFactor(Constants.WristConstants.rotations_to_degrees);

        wrist.configure(wrist_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", getWristPos());
        SmartDashboard.putNumber("Wrist Target Angle", wrist_target);
    }

    public double getWristPos() {
        return wrist_encoder.getPosition();
    }

    public void setWristPosition(double position) {
        wrist_controller.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public Command rotateWrist(double target) {
        return run(() -> setWristPosition(target));
    }

    public void setWristStatePivot(Constants.WristConstants.WristStates state) {
        switch (state) {
            case Stow:
                wrist_target = Constants.WristConstants.stow_angle;
                break;
            case Intake:
                wrist_target = Constants.WristConstants.intake_angle;
                break;
        }
        setWristPosition(wrist_target);
    }
}
