package frc.robot;

public class ShootingConstants {

  private ShootingConstants() {}

  /* ===============
   * Ballistic Constants
   * =============== */

  /// Mass of the projectile fuel (kg) - affects drag and energy loss calculations
  public static final double FUEL_MASS = 0.227;

  // Vertical velocity component (m/s) - gravitational component of projectile motion
  public static final double VERTICAL_VELOCITY_COMPONENT = 6.92858460962;

  // Square of vertical velocity component (m^2/s^2) - used in total velocity calculations
  public static final double VERTICAL_VELOCITY_COMPONENT_SQUARED = 48.0052946927;

  // Drag coefficient - exponential factor in air resistance equation
  public static final double QUADRATIC_DRAG_COEFFICIENT = 0.004997;

  // Fixed flight time (seconds) - predetermined projectile flight duration used in calculations
  public static final double MINIMUM_TIME_OF_FLIGHT = 1.15;

  /* ===============
   * Motor Adjustment Constants
   * =============== */

  // Time interval for motor adjustment (seconds) - time to recover from launch and prepare for next shot
  public static final double RELOAD_TIME = 0.07675438596;

  // Upswing velocity of projectile before launch (m/s) - pre-launch speed requirement for consistent shooting performance
  public static final double UPSWING_VELOCITY = 5.7912;

  // Moment of inertia of the shooter mechanism (kg·m^2) - affects how much torque is needed to adjust wheel speed after a shot
  public static final double SHOOTER_MOMENT_OF_INERTIA = 0.00126;

  // Resting angular velocity of the shooter wheel (rad/s) - target steady-state speed for consistent shooting performance
  public static final double FREE_ANGULAR_VELOCITY = 0;

  /** Conversion factor from radians to degrees */
  public static final double RAD_TO_DEG = 180.0 / Math.PI;
  /** Conversion factor from degrees to radians */
  public static final double DEG_TO_RAD = Math.PI / 180.0;
}
