package frc.robot.util;

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

  /** Reload time (seconds) - used for ballistic calculations with acceleration */
  private static final double R = ShootingConstants.RELOAD_TIME;

  /** Inverse of flight time (1/T) - precomputed for efficiency in calculations */
  private static final double invT = 1.0 / T;

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

  /** For debug checking */
  public static double vxFloor;

  public static double vyFloor;

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
    double dReq = Math.sqrt(xReq * xReq + yReq * yReq);
    double azReq = Math.atan2(yReq, xReq);

    // ================= ANALYTIC BASE =================
    double vFloor = (M / (K * T)) * (Math.exp((K * dReq) / M) - 1.0);
    double vx1 = vFloor * Math.cos(azReq);
    double vy1 = vFloor * Math.sin(azReq);

    vTotalNew = Math.sqrt(VY2 + vFloor * vFloor);
    thetaDeg = Math.atan(VY / vFloor) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azReq - azRad) * ShootingConstants.RAD_TO_DEG;

    // ================= ITERATIVE RK2 ROOT SOLVE =================

    // Start from analytic guess
    double vxGuess = vx1;
    double vyGuess = vy1;

    final int RK_STEPS = 20; // fixed integration resolution
    final int SOLVE_ITERS = 3; // 3–4 is ideal
    final double gain = 1.0; // Newton-like gain (usually stable)

    // Precompute
    double dt = T / RK_STEPS;

    double errX = 0.0;
    double errY = 0.0;

    for (int iter = 0; iter < SOLVE_ITERS; iter++) {

      double x = 0.0;
      double y = 0.0;
      double vx = vxGuess;
      double vy = vyGuess;

      // ===== fixed RK2 forward integration =====
      for (int i = 0; i < RK_STEPS; i++) {

        double speed = Math.sqrt(vx * vx + vy * vy);
        double ax = -ALPHA * speed * vx;
        double ay = -ALPHA * speed * vy;

        // midpoint
        double vxMid = vx + 0.5 * dt * ax;
        double vyMid = vy + 0.5 * dt * ay;
        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid);

        double axMid = -ALPHA * speedMid * vxMid;
        double ayMid = -ALPHA * speedMid * vyMid;

        // update position
        x += dt * vxMid;
        y += dt * vyMid;

        // update velocity
        vx += dt * axMid;
        vy += dt * ayMid;
      }

      // add robot motion
      double xTotal = x + xRobot;
      double yTotal = y + yRobot;

      // compute error
      errX = xTotal - xHub;
      errY = yTotal - yHub;

      // Newton-like correction
      vxGuess -= gain * errX * invT;
      vyGuess -= gain * errY * invT;
    }

    // ===== final corrected values =====

    double vFloorCorrected = Math.sqrt(vxGuess * vxGuess + vyGuess * vyGuess);
    double azCorrected = Math.atan2(vyGuess, vxGuess);

    vTotalNew = Math.sqrt(VY2 + vFloorCorrected * vFloorCorrected);
    thetaDeg = Math.atan(VY / vFloorCorrected) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azCorrected - azRad) * ShootingConstants.RAD_TO_DEG;

    long end = System.nanoTime();
    double duration = (end - start) / 1_000_000_000.0;

    double errMag = Math.sqrt(errX * errX + errY * errY);
    System.out.printf(
        "RK2 root solve -> errX=%.4f m, errY=%.4f m, errMag=%.4f m, duration=%.9f s%n",
        errX, errY, errMag, duration);
  }

  /**
   * Computes the required projectile launch parameters while compensating for robot motion.
   *
   * <p>This method performs all calculations in the floor (field) reference frame. It first
   * computes the required floor-parallel projectile velocity assuming a stationary robot, then
   * applies a Galilean transformation by subtracting the robot's floor velocity. From the corrected
   * velocity vector, it computes the required total launch speed and azimuth correction. This
   * version also accounts for robot acceleration by estimating the robot's velocity at the time of
   * projectile release.
   *
   * <p>The computed values are stored in static state variables and printed to standard output.
   *
   * @param D distance to the target along the floor (meters)
   * @param floorAzimuthDeg absolute azimuth angle of the target in the floor frame (degrees)
   * @param v_x robot floor velocity in the X direction (m/s)
   * @param v_y robot floor velocity in the Y direction (m/s)
   * @param a_x robot floor acceleration in the X direction (m/s²)
   * @param a_y robot floor acceleration in the Y direction (m/s²)
   */
  public static void computeBallistics(
      double D, double floorAzimuthDeg, double v_x, double v_y, double a_x, double a_y) {
    long start = System.nanoTime();

    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // Hub position
    double xHub = D * Math.cos(azRad);
    double yHub = D * Math.sin(azRad);

    // Compute velocity after reload time
    double v_xRelease = v_x + a_x * R;
    double v_yRelease = v_y + a_y * R;

    // Robot displacement
    double xRobot = v_xRelease * T;
    double yRobot = v_yRelease * T;

    // Required relative displacement
    double xReq = xHub - xRobot;
    double yReq = yHub - yRobot;
    double dReq = Math.sqrt(xReq * xReq + yReq * yReq);
    double azReq = Math.atan2(yReq, xReq);

    // ================= ANALYTIC BASE =================
    double vFloor = (M / (K * T)) * (Math.exp((K * dReq) / M) - 1.0);
    double vx1 = vFloor * Math.cos(azReq);
    double vy1 = vFloor * Math.sin(azReq);

    vTotalNew = Math.sqrt(VY2 + vFloor * vFloor);
    thetaDeg = Math.atan(VY / vFloor) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azReq - azRad) * ShootingConstants.RAD_TO_DEG;

    // ================= ITERATIVE RK2 ROOT SOLVE =================

    // Start from analytic guess
    double vxGuess = vx1;
    double vyGuess = vy1;

    final int RK_STEPS = 20; // fixed integration resolution
    final int SOLVE_ITERS = 3; // 3–4 is ideal
    final double gain = 1.0; // Newton-like gain (usually stable)

    // Precompute
    double dt = T / RK_STEPS;

    double errX = 0.0;
    double errY = 0.0;

    for (int iter = 0; iter < SOLVE_ITERS; iter++) {

      double x = 0.0;
      double y = 0.0;
      double vx = vxGuess;
      double vy = vyGuess;

      // ===== fixed RK2 forward integration =====
      for (int i = 0; i < RK_STEPS; i++) {

        double speed = Math.sqrt(vx * vx + vy * vy);
        double ax = -ALPHA * speed * vx;
        double ay = -ALPHA * speed * vy;

        // midpoint
        double vxMid = vx + 0.5 * dt * ax;
        double vyMid = vy + 0.5 * dt * ay;
        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid);

        double axMid = -ALPHA * speedMid * vxMid;
        double ayMid = -ALPHA * speedMid * vyMid;

        // update position
        x += dt * vxMid;
        y += dt * vyMid;

        // update velocity
        vx += dt * axMid;
        vy += dt * ayMid;
      }

      // add robot motion
      double xTotal = x + xRobot;
      double yTotal = y + yRobot;

      // compute error
      errX = xTotal - xHub;
      errY = yTotal - yHub;

      // Newton-like correction
      vxGuess -= gain * errX * invT;
      vyGuess -= gain * errY * invT;
    }

    // ===== final corrected values =====

    double vFloorCorrected = Math.sqrt(vxGuess * vxGuess + vyGuess * vyGuess);
    double azCorrected = Math.atan2(vyGuess, vxGuess);

    vTotalNew = Math.sqrt(VY2 + vFloorCorrected * vFloorCorrected);
    thetaDeg = Math.atan(VY / vFloorCorrected) * ShootingConstants.RAD_TO_DEG;
    deltaFloorAngleDeg = (azCorrected - azRad) * ShootingConstants.RAD_TO_DEG;

    long end = System.nanoTime();
    double duration = (end - start) / 1_000_000_000.0;

    double errMag = Math.sqrt(errX * errX + errY * errY);
    System.out.printf(
        "RK2 root solve -> errX=%.4f m, errY=%.4f m, errMag=%.4f m, duration=%.9f s%n",
        errX, errY, errMag, duration);
  }
}
