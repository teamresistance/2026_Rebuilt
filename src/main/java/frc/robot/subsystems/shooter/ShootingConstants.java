package frc.robot.subsystems.shooter;

public class ShootingConstants {

  private ShootingConstants() {}

  /**
   * This contains information on what angle and RPS to shoot at in a two-dimensional array. Data is
   * stored in the format of {@code [rotations per sec, hood angle]}. <br>
   * <br>
   * Usage: {@code shootingParameters[distance*10]}. <br>
   * For example, shootingParameters[67] would correlate to 6.7 meters away from the hub.
   */
  public static final double[][] params =
      new double[][] {new double[] {0, 13}}; // TODO: this is a placeholder, get actual values...

  /**
   * Contains Time of Flight values for rounded distances (to 0.5m). [2] correlates to 1m, [3] to
   * 1.5m, so on.
   */
  public static final double[] tofParams = new double[] {0}; // TODO: get values...

  /** Gets the time of flight of a shot based on distance from the time of flight array. */
  public static double getTimeOfFlight(double distance) {
    return tofParams[((int) distance) * 2];
  }

  /** Gets only the rotations per second from the parameter array. */
  public static double getRPS(double distance) {
    return params[(int) Math.round((distance) * 10)][0];
  }

  /** Gets only the hood angle from the parameter array. */
  public static double getHoodAngle(double distance) {
    return params[(int) Math.round((distance) * 10)][1];
  }

  /** Gets parameters in the format of {@code [rotations per sec, hood angle]} */
  public static double[] getParams(double distance) {
    return params[(int) Math.round((distance) * 10)];
  }
}
