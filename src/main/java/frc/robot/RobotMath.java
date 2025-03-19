package frc.robot;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class RobotMath
{

  public static class Arm
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    //TODO: Change to gear ratio of arm
    public static Angle convertAngleToSensorUnits(Angle measurement, double reductionRatio)
    {
      return Rotations.of(measurement.in(Rotations) * reductionRatio);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAngle(Angle measurement, double reductionRatio)
    {
      return Rotations.of(measurement.in(Rotations) / reductionRatio);

    }
  }

}