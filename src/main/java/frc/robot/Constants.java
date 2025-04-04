// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
// import frc.robot.RobotMath.Arm;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    public enum RobotStates {
      NORMAL,
      MANUAL,
    }
    
  }

  public static class ArmConstants {
    public static int arm_id = 16;

    public static double rotations_to_degrees = (1.0 / 120.0) * 360.0;

    public static double velocity_conversion = (360.0 / (120.0 * 60));

    public enum ArmStates {
      Stow,
      Intake
    }

    public static double stow_angle = 0;
    public static double intake_angle = -12;

  }

  public static class WristConstants {
    public static int wrist_id = 15;

    public static double rotations_to_degrees = (1.0 / 100.0) * 360.0;

    public enum WristStates {
      Stow,
      Intake
    }

    public static double stow_angle = 0;
    public static double intake_angle = 10;

  }

  public static class ElevatorConstants {
    public static final double MAX_VOLTAGE = 3.0;
    public static final double JOYSTICK_DEADBAND = 0.12;
    public static final double MAX_HEIGHT_INCHES = 46;
    public static final double MIN_HEIGHT_INCHES = 11.5;

    //check if inches are consistent
    public static double ELEVATION_GEAR_RATIO = 6.12 / (1.273 * Math.PI * 2);

    public enum ElevationTarget {
      // https://www.desmos.com/calculator/ocl2iqiu7n
      // Unit: inches
      //tune tommorow
      Stow(12),
      CoralIntake(25),
      L1(15),
      L2(17),
      L3(40.298),
      AlgaeL2(20.899),
      AlgaeL3(36.899);
  
      private double targetValue;
  
      private ElevationTarget(double targetValue) {
        this.targetValue = targetValue;
      }
  
      public double getValue() {
        return targetValue;
      }
    }
  
  }

  public static class ScoringConstants {
    public enum ScoringStates {
      Stow,
      Intake,
      L1,
      L2,
      L3,
      // AlgaeL2,
      // AlgaeL3
    }
  }

}
