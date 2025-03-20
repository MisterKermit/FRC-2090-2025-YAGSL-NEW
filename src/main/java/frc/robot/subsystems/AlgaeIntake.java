package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    //TODO: tune canid
    private TalonFX algae_intake = new TalonFX(19); 
    
    private static TalonFXConfiguration config = new TalonFXConfiguration();
    static {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    private double intakeAlgaeSpeed = 0.5;

    public AlgaeIntake() {
        algae_intake.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        
    }

    public Command runIntakeAlgae() {
        return run(() -> algae_intake.set(intakeAlgaeSpeed));
    }

    public Command reverseIntakeAlgae() {
        return run(() -> algae_intake.set(-intakeAlgaeSpeed));
    }

}
