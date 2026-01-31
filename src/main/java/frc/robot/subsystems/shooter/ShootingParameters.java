package frc.robot.subsystems.shooter;

public class ShootingParameters {

  /**
   * This contains information on what angle and RPS to shoot at in a two-dimensional array. Data is
   * stored in the format of {@code [rotations per sec, hood angle]}. <br>
   * <br>
   * Usage: {@code shootingParameters[distance*10]}. <br>
   * For example, shootingParameters[67] would correlate to 6.7 meters away from the hub.
   */
  public static final double[][] params = new double[][] {new double[] {0, 13}}; // TODO: this is a placeholder, get actual values...
}
