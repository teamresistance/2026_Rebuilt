package frc.robot.util;

import frc.robot.Constants;

/**
 * Utility methods for converting between motor revolutions and shooter angles, based on the known
 * gear ratios of the shooter mechanism. These methods allow commands and subsystems to work in
 * intuitive angle units while the ShooterIO implementation can work in motor revolutions,
 * abstracting away the conversion details. The conversion factors are defined in {@code Constants}
 * and are based on the physical design of the shooter mechanisms, (e.g., how many motor revolutions
 * correspond to one degree of hood pitch or turret yaw).
 */
public class Conversions {
  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the angle converted to an equal amount of motor revolutions. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodRevs(double degrees) {
    return degrees * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of yaw.
   */
  public static double toTurretDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  /**
   * Returns the angle converted to an equal amount of motor revolutions. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of yaw.
   */
  public static double toTurretRevs(double degrees) {
    return degrees * Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }
}
