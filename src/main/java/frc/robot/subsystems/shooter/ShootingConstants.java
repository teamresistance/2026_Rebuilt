package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingConstants {

  private ShootingConstants() {}

  /** Sets up the interpolating maps with all recorded data points */
  public static void configureShootingConstants() {

    //    distanceToRpsMap.put(1.65, 45.0);
    //    distanceToRpsMap.put(1.79, 46.0);
    //    distanceToRpsMap.put(1.96, 46.0);
    //    distanceToRpsMap.put(2.17, 50.0);
    //    distanceToRpsMap.put(2.33, 50.0);
    //    // BEGIN GOOD
    //    distanceToRpsMap.put(2.51, 47.2);
    //    distanceToRpsMap.put(2.69, 47.2);
    //    distanceToRpsMap.put(3.03, 48.8);
    //    distanceToRpsMap.put(3.29, 50.0);
    //    distanceToRpsMap.put(3.5, 51.3);
    //    distanceToRpsMap.put(3.72, 52.6);
    //    distanceToRpsMap.put(4.27, 53.0);
    //    // END GOOD
    //    distanceToRpsMap.put(4.54, 57.0);
    //    distanceToRpsMap.put(4.71, 57.0);
    //    distanceToRpsMap.put(4.99, 58.5);
    //    distanceToRpsMap.put(5.35, 58.5);
    //    distanceToRpsMap.put(5.59, 60.3);
    //    distanceToRpsMap.put(5.94, 62.0);
    //
    //    distanceToHoodMap.put(1.65, 17.5);
    //    distanceToHoodMap.put(1.79, 17.5);
    //    distanceToHoodMap.put(1.96, 17.5);
    //    distanceToHoodMap.put(2.17, 17.5);
    //    distanceToHoodMap.put(2.33, 18.0);
    //    distanceToHoodMap.put(2.51, 20.0);
    //    distanceToHoodMap.put(2.82, 22.0);
    //    distanceToHoodMap.put(3.05, 23.0);
    //    distanceToHoodMap.put(3.3, 23.0);
    //    distanceToHoodMap.put(3.5, 24.0);
    //    distanceToHoodMap.put(3.72, 26.0);
    //    distanceToHoodMap.put(4.27, 27.0);
    //    distanceToHoodMap.put(4.54, 27.0);
    //    distanceToHoodMap.put(4.71, 28.0);
    //    distanceToHoodMap.put(4.99, 29.5);
    //    distanceToHoodMap.put(5.35, 30.0);
    //    distanceToHoodMap.put(5.59, 31.0);
    //    distanceToHoodMap.put(5.94, 31.0);
    //
    //    distanceToTimeOfFlightMap.put(2.98, 1.4);
    //    distanceToTimeOfFlightMap.put(2.34, 1.5);
    //    distanceToTimeOfFlightMap.put(3.8, 1.6);
    //    distanceToTimeOfFlightMap.put(1.59, 1.14);
    //    distanceToTimeOfFlightMap.put(4.9, 1.84);

    distanceToHoodMap.put(1.46, 17.5);
    distanceToHoodMap.put(1.84, 17.5);
    distanceToHoodMap.put(2.23, 17.5);
    distanceToHoodMap.put(2.52, 17.8);
    distanceToHoodMap.put(2.86, 19.0);
    distanceToHoodMap.put(3.23, 20.5);
    distanceToHoodMap.put(3.54, 22.0);
    distanceToHoodMap.put(3.88, 24.0);
    distanceToHoodMap.put(4.18, 26.0);
    distanceToHoodMap.put(4.48, 27.0);
    distanceToHoodMap.put(4.83, 28.0);
    distanceToHoodMap.put(5.11, 29.0);
    distanceToHoodMap.put(5.52, 30.0);
    distanceToHoodMap.put(5.89, 32.0);

    distanceToRpsMap.put(1.46, 41.0 - 2);
    distanceToRpsMap.put(1.84, 45.0 - 2);
    distanceToRpsMap.put(2.23, 48.0 - 2);
    distanceToRpsMap.put(2.52, 49.7 - 2.75);
    distanceToRpsMap.put(2.86, 49.7 - 2.75);
    distanceToRpsMap.put(3.23, 50.4 - 1.5);
    distanceToRpsMap.put(3.54, 50.9 - 0.5);
    distanceToRpsMap.put(3.88, 51.5);
    distanceToRpsMap.put(4.18, 52.2);
    distanceToRpsMap.put(4.48, 53.2);
    distanceToRpsMap.put(4.83, 55.0);
    distanceToRpsMap.put(5.11, 56.8);
    distanceToRpsMap.put(5.52, 59.6);
    distanceToRpsMap.put(5.89, 61.0);

    distanceToTimeOfFlightMap.put(2.51, 1.22);
    distanceToTimeOfFlightMap.put(1.56, 1.11);
    distanceToTimeOfFlightMap.put(2.52, 1.12);
    distanceToTimeOfFlightMap.put(3.06, 1.3);
    distanceToTimeOfFlightMap.put(3.86, 1.34);
    distanceToTimeOfFlightMap.put(4.25, 1.26);
    distanceToTimeOfFlightMap.put(4.77, 1.32);
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
