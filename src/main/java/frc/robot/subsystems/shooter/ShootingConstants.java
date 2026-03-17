package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingConstants {

  private ShootingConstants() {}

  /** Sets up the interpolating maps with all recorded data points */
  public static void configureShootingConstants() {
    //    distanceToRpsMap.put(1.35, 40.0);
    //    distanceToRpsMap.put(1.85, 43.8);
    //    distanceToRpsMap.put(2.2, 44.0);
    //    distanceToRpsMap.put(2.7, 45.6);
    //    distanceToRpsMap.put(3.25, 49.7);
    //    distanceToRpsMap.put(3.74, 51.0);
    //    distanceToRpsMap.put(4.4, 54.0);
    //    distanceToRpsMap.put(5.0, 57.0);
    //
    //    distanceToHoodMap.put(1.85, 17.5);
    //    distanceToHoodMap.put(1.35, 17.5);
    //    distanceToHoodMap.put(2.2, 19.0);
    //    distanceToHoodMap.put(2.7, 22.0);
    //    distanceToHoodMap.put(3.25, 24.0);
    //    distanceToHoodMap.put(3.74, 25.0);
    //    distanceToHoodMap.put(4.4, 27.0);
    //    distanceToHoodMap.put(5.0, 29.0);

    distanceToRpsMap.put(1.65, 45.0);
    distanceToRpsMap.put(1.79, 46.0);
    distanceToRpsMap.put(1.96, 46.0);
    distanceToRpsMap.put(2.17, 50.0);
    distanceToRpsMap.put(2.33, 50.0);
    distanceToRpsMap.put(2.51, 50.0);
    distanceToRpsMap.put(2.82, 51.0);
    distanceToRpsMap.put(3.3, 51.0);
    distanceToRpsMap.put(3.72, 53.0);
    distanceToRpsMap.put(4.27, 54.5);
    distanceToRpsMap.put(4.71, 57.0);
    distanceToRpsMap.put(5.35, 58.5);

    distanceToHoodMap.put(1.65, 17.5);
    distanceToHoodMap.put(1.79, 17.5);
    distanceToHoodMap.put(1.96, 17.5);
    distanceToHoodMap.put(2.17, 17.5);
    distanceToHoodMap.put(2.33, 18.0);
    distanceToHoodMap.put(2.51, 20.0);
    distanceToHoodMap.put(2.82, 22.0);
    distanceToHoodMap.put(3.3, 23.0);
    distanceToHoodMap.put(3.72, 26.0);
    distanceToHoodMap.put(4.27, 27.0);
    distanceToHoodMap.put(4.71, 28.0);
    distanceToHoodMap.put(5.35, 30.0);

    distanceToTimeOfFlightMap.put(2.98, 1.4);
    distanceToTimeOfFlightMap.put(2.34, 1.5);
    distanceToTimeOfFlightMap.put(3.8, 1.6);
    distanceToTimeOfFlightMap.put(1.59, 1.14);
    distanceToTimeOfFlightMap.put(4.9, 1.84);
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
