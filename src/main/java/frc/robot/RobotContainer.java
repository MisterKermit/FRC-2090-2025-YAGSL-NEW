// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.RobotStates;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevationTarget;
import frc.robot.Constants.ScoringConstants.ScoringStates;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ManualElevationCommand;
import frc.robot.commands.ScoringMacro;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.management.OperatingSystemMXBean;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController driverXbox = new CommandXboxController(0);

  public final static CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final ScoringSubsystem scoring = new ScoringSubsystem();

  private final AlgaeIntake aIntake = new AlgaeIntake();

  private final CoralIntake cIntake = new CoralIntake();

  private final VisionSubsystem vision = new VisionSubsystem("limelight");

  // private final Hang hang = new Hang();

  private static RobotStates currentRoboState = RobotStates.NORMAL;

  private static boolean slowMode = false;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -0.5,
      () -> driverXbox.getLeftX() * -0.5)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND) 
      .scaleTranslation(slowMode || scoring.returnState() != ScoringStates.Stow ? 1 : 0.5)
      .allianceRelativeControl(true);

  SwerveInputStream driveSlowAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -0.1,
      () -> driverXbox.getLeftX() * -0.1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1)
      .allianceRelativeControl(true);
  /*

   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  public Command IntakeSETPOS() {
    return scoring.setArmPivotStateCommand(ScoringStates.Intake).andThen(new ElevatorCommand(elevator, ElevationTarget.CoralIntake.getValue()));
  }

  public Command slowModeToggle(boolean isSlow) {
    return new InstantCommand(() -> slowMode = isSlow ? !isSlow : isSlow);
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Currently Broken
//     NamedCommands.registerCommand("extendElevatorToL3", elevator.elevateCommandState(ElevatorSubsystem.ElevationTarget.L3));
//     NamedCommands.registerCommand("retractElevatorToMin", elevator.elevateCommandState(Constants.ElevatorConstants.MIN_HEIGHT_INCHES));
//     NamedCommands.registerCommand("extendElevatorToCoralIntake", elevator.elevateCommandState(ElevatorSubsystem.ElevationTarget.CoralIntake));

//     // following is for scoring (arm and wrist are currently seperate)
//     NamedCommands.registerCommand("scoringStow",
//     new ParallelCommandGroup(
//         scoring.setArmPivotStateCommand(ScoringConstants.ScoringStates.Stow),
//         Commands.runOnce(() -> scoring.setWristState(ScoringConstants.ScoringStates.Stow))
//         )
//     );

//     NamedCommands.registerCommand("scoringIntake",
//     new ParallelCommandGroup(
//         scoring.setArmPivotStateCommand(ScoringConstants.ScoringStates.Intake),
//         Commands.runOnce(() -> scoring.setWristState(ScoringConstants.ScoringStates.Intake))
//         )
//     );

// // THE FOLLOWING IS FOR THE L3 SCORING STATE THAT HAS NOT YET BEEN MADE YET
//     NamedCommands.registerCommand("scoringL3",
//     new ParallelCommandGroup(
//         scoring.setArmPivotStateCommand(ScoringConstants.ScoringStates.L3),
//         Commands.runOnce(() -> scoring.setWristState(ScoringConstants.ScoringStates.L3))
//         )
//     );

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    
    // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveSlowFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveSlowAngularVelocity);
    // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    Command ManualControlElevator = new ManualElevationCommand(elevator, driverXbox);
    // Command ManualArm = arm.manualArm(driverXbox);
    // Command HangSequence = hang.hangCommand(driverXbox);

    
    // if (slowMode || scoring.returnState() != ScoringStates.Stow) {
    //   drivebase.setDefaultCommand(driveSlowFieldOrientedAngularVelocity);
    // } else {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    // Command SetArmPivot = arm.setStateArm(ArmStates.Stow);
    // Command ArmReset = arm.moveArmToPosition(0);
    // Command ArmIntake = arm.rotateArm(0.5);
    // Command RotateArmTest = arm.rotateArm(40);

    // Command ArmUp = scoring.wristForward();
    // Command ArmDown = arm.runArmDown();

    Trigger YAxisJoystickTrigger = new Trigger(() -> {
      if (MathUtil.applyDeadband(operatorXbox.getLeftY(), 0.01) > 0.01|| 
          MathUtil.applyDeadband(operatorXbox.getLeftY(), 0.01) < -0.01 ||
          MathUtil.applyDeadband(operatorXbox.getRightY(), 0.01) > 0.01 || 
          MathUtil.applyDeadband(operatorXbox.getRightY(), 0.01) < -0.01) {
        return true;
      } else {
        return false;
      }
    });

    driverXbox.x().onTrue(new ScoringMacro(scoring, elevator, ScoringStates.Intake));

    switch (currentRoboState) {
      case NORMAL:
        operatorXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        operatorXbox.b().whileTrue(cIntake.runCoralIntake());
        operatorXbox.x().whileTrue(cIntake.reverseCoralIntake());
        operatorXbox.y().onTrue(new DriveToAprilTag(drivebase, vision));
        operatorXbox.leftBumper().whileTrue(aIntake.runAlgaeIntake());
        operatorXbox.rightBumper().whileTrue(aIntake.reverseAlgaeIntake());
        // driverXbox.b().onTrue(new ScoringMacro(scoring, elevator, ScoringStates.Stow));
        driverXbox.y().onTrue(IntakeSETPOS());
        // driverXbox.x().onTrue(new ElevatorCommand(elevator,ElevationTarget.L1.getValue()));
        driverXbox.leftBumper().onTrue(slowModeToggle(slowMode));
        // operatorXbox.povUp()
        //   .whileTrue(new InstantCommand(() -> hang.setPower(-0.5)))
        //   .whileFalse(new InstantCommand(() -> hang.setPower(0)));
        driverXbox.back().onTrue(new InstantCommand(() -> currentRoboState = RobotStates.MANUAL));
        operatorXbox.back().onTrue(new InstantCommand(() -> currentRoboState = RobotStates.MANUAL));
        break;

      case MANUAL:
        operatorXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().onTrue(new InstantCommand(() -> currentRoboState = RobotStates.NORMAL));
        operatorXbox.back().onTrue(new InstantCommand(() -> currentRoboState = RobotStates.NORMAL))
        YAxisJoystickTrigger
        .onTrue(scoring.setManualArmVoltage(() -> MathUtil.applyDeadband(-operatorXbox.getLeftY(), 0.01), 
                                                  () -> MathUtil.applyDeadband(-operatorXbox.getRightY(), 0.01)))
        .onFalse(scoring.stopWholeArm());
        driverXbox.leftBumper().onTrue(slowModeToggle(slowMode));
        break;
    }
      

    

    
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    
    // test bind
    // driverXbox.b().onTrue(elevator.elevateCommand(ElevationTarget.L1));
    // driverXbox.x().onTrue(elevator.elevateCommand(ElevationTarget.L2));
    // driverXbox.povUp().whileTrue(HangSequence);
    // driverXbox.y().onTrue(elevator.elevateCommand(ElevationTarget.CoralIntake));
    // driverXbox.y().onTrue(RotateArmTest);
    // driverXbox.y().onTrue(new ScoringMacro(arm, elevator, wrist, ScoringStates.Intake));
    // driverXbox.x().onTrue(SetArmPivot);
    // driverXbox.y().onTrue(ArmUp);  
    // driverXbox.y().onTrue(ArmDown);
    // driverXbox.leftBumper().whileTrue(ArmIntake);
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }
// // command for autonomous
//   public Command getAutonomousCommand() {
//     return new FollowPathAutoCommand(swerveSubsystem);
//   }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}