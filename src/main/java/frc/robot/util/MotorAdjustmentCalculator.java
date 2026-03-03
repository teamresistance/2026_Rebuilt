package frc.robot.util;

import frc.robot.Constants.ShootingConstants;

/**
 * MotorAdjustmentCalculator - Computes motor adjustments to compensate for energy loss during
 * projectile launch.
 *
 * <p>This class calculates the required angular acceleration of the launcher wheel needed to
 * maintain consistent performance after each projectile is fired, accounting for: - Energy lost
 * from fuel ejection
 */
public class MotorAdjustmentCalculator {

  public static double restingAngularVelocity = ShootingConstants.FREE_ANGULAR_VELOCITY;
  public static double R = ShootingConstants.SHOOTER_RADIUS;
  public static double E = ShootingConstants.SHOOTER_EFFICIENCY;
  public static double T = ShootingConstants.RELOAD_TIME;

  /* =======================
   * Output State Variables
   * ======================= */

  /** Required angular acceleration of launcher motor (rad/s²) - compensates for energy loss */
  public static double desiredAngularAcceleration = 0;

  /**
   * Desired angular velocity of launcher wheel after adjustment (rad/s) - target speed post-launch
   */
  public static double desiredAngularVelocity = 0;

  /**
   * Computes the required motor angular acceleration to restore launcher wheel speed.
   *
   * <p>This method calculates the angular acceleration needed to compensate for energy lost when
   * the projectile is launched. It retrieves the actual launch velocity from the ballistic
   * calculator, then uses rotational dynamics to determine the motor adjustment required to return
   * the wheel from its post-launch state back to resting angular velocity within the specified time
   * interval.
   *
   * <p>The computed angular acceleration is stored in {@link #desiredAngularAcceleration} and
   * relevant metrics are printed to standard output.
   */
  public static void computeMotorAdjustment(double launchSpeed) {

    long start = System.nanoTime();

    desiredAngularVelocity = launchSpeed / (E * R) / 2 / Math.PI;

    desiredAngularAcceleration = (desiredAngularVelocity - restingAngularVelocity) / T;

    long end = System.nanoTime();
    // TODO: Log start/end times
  }
}
