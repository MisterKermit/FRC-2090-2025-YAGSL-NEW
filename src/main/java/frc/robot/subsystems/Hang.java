package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Hang extends SubsystemBase {
  private final SparkMax hang1;
  private final SparkMax hang2;
  private final RelativeEncoder encoder1;
  private final RelativeEncoder encoder2;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;

  public Hang() {
    // Initialize motors
    hang1 = new SparkMax(18, MotorType.kBrushless);
    hang2 = new SparkMax(19, MotorType.kBrushless);

    // Encoders
    encoder1 = hang1.getEncoder();
    encoder2 = hang2.getEncoder();

    // Create config objects
    SparkMaxConfig hang1Config = new SparkMaxConfig();
    SparkMaxConfig hang2Config = new SparkMaxConfig();

    // Idle mode
    hang1Config.idleMode(IdleMode.kBrake);
    hang2Config.idleMode(IdleMode.kBrake);

    // Limit switches
    hang1Config.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    hang2Config.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // Example soft limits for hang1
    hang1Config.softLimit
        .forwardSoftLimit(30)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-30)
        .reverseSoftLimitEnabled(true);

    // Apply configurations
    hang1.configure(hang1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hang2.configure(hang2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Zero the encoders
    encoder1.setPosition(0);
    encoder2.setPosition(0);

    // Grab limit switch references (if you want to display them)
    forwardLimitSwitch = hang1.getForwardLimitSwitch();
    reverseLimitSwitch = hang1.getReverseLimitSwitch();

    SmartDashboard.setDefaultBoolean("Direction", true);
  }

// set power method
  public void setPower(double power) {
    hang1.set(power);
    hang2.set(-power);
  }

  @Override
  public void periodic() {
    // Display data
    SmartDashboard.putBoolean("Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Reached", reverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Hang1 Output", hang1.getAppliedOutput());
    SmartDashboard.putNumber("Hang2 Output", hang2.getAppliedOutput());
    SmartDashboard.putNumber("Encoder1 Position", encoder1.getPosition());
    SmartDashboard.putNumber("Encoder2 Position", encoder2.getPosition());
  }

  public Command hangCommand(CommandXboxController controller) {
    return new HangCommand(this, controller);
  }

  /** 
   * When the D-pad is pressed up (POV = 0°), the motors run at +0.5 power.
   * When pressed down (POV = 180°), they run at -0.5 power.
   * Otherwise, the motors are stopped.
   */
  public static class HangCommand extends Command {
    private final Hang hang;
    private final CommandXboxController controller;

    public HangCommand(Hang hang, CommandXboxController controller) {
      this.hang = hang;
      this.controller = controller;
      addRequirements(hang);
    }

    @Override
    public void execute() {
      // Get the D-pad (POV) angle
      int pov = controller.getHID().getPOV();
      double power = 0.0;
      
      if (pov == 0) {         // D-pad up
        power = 0.5;
      } else if (pov == 180) { // D-pad down
        power = -0.5;
      } else {
        power = 0.0;
      }
      
      hang.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
      hang.setPower(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}