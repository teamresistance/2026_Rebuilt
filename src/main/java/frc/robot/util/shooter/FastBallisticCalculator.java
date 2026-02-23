package frc.robot.util.shooter;

import frc.robot.Constants.ShootingConstants;

/**
 * FastBallisticCalculator - Calculates projectile launch parameters with air resistance.
 *
 * <p>This class computes the required launch velocity and angle for a projectile to hit a target at
 * a specified distance, accounting for: - Exponential air drag force proportional to distance
 * traveled - Vertical gravitational component - Robot platform velocity offset (converts target
 * frame to robot frame)
 *
 * <p>The computation uses physics equations for projectile motion with quadratic drag, solving for
 * launch velocity magnitude and direction corrections relative to the robot's velocity. -Samuel
 * Sapatla, Mihir Sonthi
 */
public final class FastBallisticCalculator {

  // Constants
  /** Vertical velocity component (m/s) - gravitational component of projectile */
  private static final double VY = ShootingConstants.VERTICAL_VELOCITY_COMPONENT;

  /** Square of vertical velocity component (VY^2) - used for total velocity calculations */
  private static final double VY2 = ShootingConstants.VERTICAL_VELOCITY_COMPONENT_SQUARED;

  /** Mass coefficient (kg) - affects drag acceleration calculations */
  public static final double M = ShootingConstants.FUEL_MASS;

  /** Drag coefficient - exponential factor in air resistance equation */
  private static final double K = ShootingConstants.QUADRATIC_DRAG_COEFFICIENT;

  /** Fixed flight time (seconds) - predetermined projectile flight duration */
  private static final double T = ShootingConstants.MINIMUM_TIME_OF_FLIGHT;

  /** Alpha constant for RK2 correction - derived from drag coefficient and mass */
  private static final double ALPHA = K / M;

  /*
   * =======================
   * Output State Variables
   * =======================
   */

  /** Final total launch velocity magnitude (m/s) */
  public static double vTotalNew;

  /** Vertical elevation angle above the floor (degrees) */
  public static double thetaDeg;

  /** Floor azimuth (degrees) - absolute angle of the target in the floor frame */
  public static double floorAzimuthDeg;

  /** Required correction to the floor azimuth (degrees) */
  public static double deltaFloorAngleDeg;

  /**
   * Computes the required projectile launch parameters while compensating for robot motion.
   *
   * <p>This method performs all calculations in the floor (field) reference frame. It first
   * computes the required floor-parallel projectile velocity assuming a stationary robot, then
   * applies a Galilean transformation by subtracting the robot's floor velocity. From the corrected
   * velocity vector, it computes the required total launch speed and azimuth correction.
   *
   * <p>The computed values are stored in static state variables and printed to standard output.
   *
   * @param D distance to the target along the floor (meters)
   * @param floorAzimuthDeg absolute azimuth angle of the target in the floor frame (degrees)
   * @param v_x robot floor velocity in the X direction (m/s)
   * @param v_y robot floor velocity in the Y direction (m/s)
   */
  public static void computeBallistics(double D, double floorAzimuthDeg, double v_x, double v_y) {
    long start = System.nanoTime();

    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // Hub position
    double xHub = D * Math.cos(azRad);
    double yHub = D * Math.sin(azRad);

    // Robot displacement
    double xRobot = v_x * T;
    double yRobot = v_y * T;

    // Required relative displacement
    double xReq = xHub - xRobot;
    double yReq = yHub - yRobot;
    double dReq = Math.hypot(xReq, yReq);
    double azReq = Math.atan2(yReq, xReq);

    // ================= ANALYTIC BASE =================
    double vFloor = (M / (K * T)) * (Math.exp((K * dReq) / M) - 1.0);
    double vx1 = vFloor * Math.cos(azReq);
    double vy1 = vFloor * Math.sin(azReq);

    vTotalNew = Math.sqrt(VY2 + vFloor * vFloor);
    thetaDeg = Math.atan(VY / vFloor) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azReq - azRad) * ShootingConstants.RAD_TO_DEG;

    // ================= ADAPTIVE RK2 CORRECTION =================
    double x = 0.0, y = 0.0;
    double vx = vx1, vy = vy1;

    double tAccum = 0.0;
    double dtMax = T / 10.0; // max step for RoboRIO speed
    double dtMin = T / 100.0; // minimum step to avoid zero
    double tol = 0.001; // target position tolerance (meters)

    while (tAccum < T) {
      double dt = Math.min(dtMax, T - tAccum);

      // compute midpoint RK2 step
      double vxMid = vx / (1 + 0.5 * ALPHA * dt * Math.abs(vx));
      double vyMid = vy / (1 + 0.5 * ALPHA * dt * Math.abs(vy));

      double xStep = dt * vxMid;
      double yStep = dt * vyMid;

      double vxNew = vx / (1 + ALPHA * dt * Math.abs(vxMid));
      double vyNew = vy / (1 + ALPHA * dt * Math.abs(vyMid));

      // estimate local error with half-step
      double vxHalf = vx, vyHalf = vy;
      double xHalf = 0.0, yHalf = 0.0;
      double dtHalf = dt / 2.0;

      for (int i = 0; i < 2; i++) {
        double vxMidHalf = vxHalf / (1 + 0.5 * ALPHA * dtHalf * Math.abs(vxHalf));
        double vyMidHalf = vyHalf / (1 + 0.5 * ALPHA * dtHalf * Math.abs(vyHalf));

        xHalf += dtHalf * vxMidHalf;
        yHalf += dtHalf * vyMidHalf;

        vxHalf = vxHalf / (1 + ALPHA * dtHalf * Math.abs(vxMidHalf));
        vyHalf = vyHalf / (1 + ALPHA * dtHalf * Math.abs(vyMidHalf));
      }

      double err = Math.hypot((xStep - xHalf), (yStep - yHalf));

      // adapt dt
      if (err > tol && dt > dtMin) {
        dt *= 0.5; // reduce step
        continue;
      } else if (err < tol / 4 && dt * 2 <= dtMax) {
        dt *= 2.0; // increase step
      }

      // accept step
      x += xStep;
      y += yStep;
      vx = vxNew;
      vy = vyNew;
      tAccum += dt;
    }

    x += xRobot;
    y += yRobot;

    // Corrective gain for final tiny adjustment
    double errX = x - xHub;
    double errY = y - yHub;
    double gain = 0.9;
    vx1 -= gain * errX / T;
    vy1 -= gain * errY / T;

    double vFloorCorrected = Math.hypot(vx1, vy1);
    double azCorrected = Math.atan2(vy1, vx1);

    vTotalNew = Math.sqrt(VY2 + vFloorCorrected * vFloorCorrected);
    thetaDeg = Math.atan(VY / vFloorCorrected) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azCorrected - azRad) * ShootingConstants.RAD_TO_DEG;

    long end = System.nanoTime();
    double duration = (end - start) / 1_000_000_000.0;

    double errMag = Math.hypot(errX, errY);
    System.out.printf(
        "Adaptive RK2 -> errX=%.4f m, errY=%.4f m, errMag=%.4f m, duration=%.9f s%n",
        errX, errY, errMag, duration);
  }
}
