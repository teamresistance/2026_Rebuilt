package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingMaps {

  private ShootingMaps() {}

  /** Sets up the interpolating maps with all recorded data points */
  public static void configureShootingMaps() {
    distanceToRpsMap.put(1.0, 1.0);

    distanceToHoodMap.put(2.0, 2.0);

    // placeholder values TODO: ts
    distanceToTimeOfFlightMap.put(1.0, 0.9);
    distanceToTimeOfFlightMap.put(2.0, 1.0);
    distanceToTimeOfFlightMap.put(3.0, 1.1);
    distanceToTimeOfFlightMap.put(4.0, 1.2);
    distanceToTimeOfFlightMap.put(5.0, 1.3);
  }

  private static final InterpolatingDoubleTreeMap distanceToRpsMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToHoodMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /** Gets time of flight for a set distance */
  public static double getTimeOfFlight(double distance) {
    return distanceToTimeOfFlightMap.get(distance);
  }

  /** Gets the rotations per second for a set distance */
  public static double getRPS(double distance) {
    return distanceToRpsMap.get(distance);
  }

  /** Gets the hood angle for a set distance */
  public static double getHoodAngle(double distance) {
    return distanceToHoodMap.get(distance);
  }
}
