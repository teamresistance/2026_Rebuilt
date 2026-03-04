package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
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

  private static final double HUB_HEIGHT = ShootingConstants.HUB_HEIGHT;

  /*
   * =======================
   * Output State Variables
   * =======================
   */

  /** Floor azimuth (degrees) - absolute angle of the target in the floor frame */
  public static double floorAzimuthDeg;

  /** For debug checking */
  public static double vxFloor;

  public static double vyFloor;

  public static Pose2d robotPose;

  private static final int RK_STEPS = 30; // Increased for sub-centimeter precision
  private static final int SOLVE_ITERS = 3;
  private static final double dt = T / RK_STEPS;
  private static final double G = 9.806;

  public record BallisticSolution(
      double launchSpeed, double hoodAngleDeg, double deltaAzimuthDeg) {}

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
  public static BallisticSolution computeBallistics(
      double D, double floorAzimuthDeg, double v_x, double v_y) {
    long start = System.nanoTime();

    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // 1. Hub position is fixed in the Field Frame
    double xHub = D * Math.cos(azRad);
    double yHub = D * Math.sin(azRad);

    // 2. ANALYTIC BASE (Initial guess to hit a stationary target distance D)
    double vFloorField = (M / (K * T)) * (Math.exp((K * D) / M) - 1.0);
    double vxGuess = vFloorField * Math.cos(azRad);
    double vyGuess = vFloorField * Math.sin(azRad);
    double vzGuess = (VY + 0.5 * 9.806 * T); // Initial vertical guess to counter gravity

    // 3. Optimized Iterative Solve
    final double dt = T / RK_STEPS;

    for (int iter = 0; iter < SOLVE_ITERS; iter++) {
      double x = 0.0, y = 0.0, z = 0.0;
      double vx = vxGuess, vy = vyGuess, vz = vzGuess;

      for (int i = 0; i < RK_STEPS; i++) {
        // Instantaneous 3D speed (The "True" Drag source)
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double dragFactor = -ALPHA * speed;

        // RK2 Midpoint
        double vxMid = vx + 0.5 * dt * (dragFactor * vx);
        double vyMid = vy + 0.5 * dt * (dragFactor * vy);
        double vzMid = vz + 0.5 * dt * (dragFactor * vz - G);

        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);
        double dragFactorMid = -ALPHA * speedMid;

        // State Update
        x += dt * vxMid;
        y += dt * vyMid;
        z += dt * vzMid;

        vx += dt * (dragFactorMid * vxMid);
        vy += dt * (dragFactorMid * vyMid);
        vz += dt * (dragFactorMid * vzMid - G);
      }

      // Compute 3D Error
      double errX = x - xHub;
      double errY = y - yHub;
      double errZ = z - HUB_HEIGHT;

      // Newton correction (using invT for horizontal,
      // vertical requires less gain due to gravity's dominance)
      vxGuess -= errX * invT;
      vyGuess -= errY * invT;
      vzGuess -= errZ * invT;
    }

    // 4. GALILEAN TRANSFORMATION (The "Magic" Step)
    // vxGuess is what the ball needs to be doing relative to the ground.
    // The shooter only provides the difference between that and the robot's velocity.
    double vxShooter = vxGuess - v_x;
    double vyShooter = vyGuess - v_y;

    double vFloorCorrected = Math.sqrt(vxShooter * vxShooter + vyShooter * vyShooter);

    // vTotalNew now uses the specific vzGuess that hit the hub height
    double vTotalNew = Math.sqrt(vFloorCorrected * vFloorCorrected + vzGuess * vzGuess);
    double thetaDeg = Math.atan2(vzGuess, vFloorCorrected) * ShootingConstants.RAD_TO_DEG;
    double deltaFloorAngleDeg =
        (Math.atan2(vyShooter, vxShooter) - azRad) * ShootingConstants.RAD_TO_DEG;

    return new BallisticSolution(vTotalNew, thetaDeg, deltaFloorAngleDeg);
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
  public static BallisticSolution computeBallistics(
      double D, double floorAzimuthDeg, double v_x, double v_y, double a_x, double a_y) {
    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // 1. PROJECT ROBOT AND TARGET RELATIVE TO RELEASE POINT
    // Robot displacement from now until release (t=R)
    double xRel = v_x * R + 0.5 * a_x * R * R;
    double yRel = v_y * R + 0.5 * a_y * R * R;

    // Robot velocity at release
    double vxRobotRelease = v_x + a_x * R;
    double vyRobotRelease = v_y + a_y * R;

    // Hub position relative to the START position (field frame)
    double xHubField = D * Math.cos(azRad);
    double yHubField = D * Math.sin(azRad);

    // Target displacement relative to the release point
    double dxTarget = xHubField - xRel;
    double dyTarget = yHubField - yRel;
    double dRelative = Math.sqrt(dxTarget * dxTarget + dyTarget * dyTarget);

    // 2. INITIAL GUESS (Field Frame)
    // We use the relative distance to get a better starting ground-velocity guess
    double vFloorGuess = (M / (K * T)) * (Math.exp((K * dRelative) / M) - 1.0);
    double vxGuess = vFloorGuess * (dxTarget / dRelative);
    double vyGuess = vFloorGuess * (dyTarget / dRelative);
    double vzGuess = (HUB_HEIGHT / T) + (0.5 * 9.806 * T); // Simple gravity compensation

    // 3. SOLVE LOOP
    for (int iter = 0; iter < 4; iter++) { // 4 iterations to settle quadratic error
      double x = 0.0, y = 0.0, z = 0.0;
      double vx = vxGuess, vy = vyGuess, vz = vzGuess;

      for (int i = 0; i < RK_STEPS; i++) {
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double dragFactor = -ALPHA * speed;

        // RK2 Midpoint
        double vxMid = vx + 0.5 * dt * (dragFactor * vx);
        double vyMid = vy + 0.5 * dt * (dragFactor * vy);
        double vzMid = vz + 0.5 * dt * (dragFactor * vz - G);

        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);
        double dragMid = -ALPHA * speedMid;

        x += dt * vxMid;
        y += dt * vyMid;
        z += dt * vzMid;
        vx += dt * (dragMid * vxMid);
        vy += dt * (dragMid * vyMid);
        vz += dt * (dragMid * vzMid - G);
      }

      // Error relative to the hub's position from the release point
      double errX = x - dxTarget;
      double errY = y - dyTarget;
      double errZ = z - HUB_HEIGHT;

      // Apply corrections
      vxGuess -= errX * invT;
      vyGuess -= errY * invT;
      vzGuess -= errZ * invT;
    }

    // 4. TRANSFORM TO ROBOT FRAME
    double vxShooter = vxGuess - vxRobotRelease;
    double vyShooter = vyGuess - vyRobotRelease;

    double vFloorFinal = Math.sqrt(vxShooter * vxShooter + vyShooter * vyShooter);
    double vTotalNew = Math.sqrt(vFloorFinal * vFloorFinal + vzGuess * vzGuess);
    double thetaDeg = Math.atan2(vzGuess, vFloorFinal) * ShootingConstants.RAD_TO_DEG;

    // Azimuth is the angle the shooter must face relative to the field 0
    double azimuthShooter = Math.atan2(vyShooter, vxShooter);
    double deltaFloorAngleDeg = (azimuthShooter - azRad) * ShootingConstants.RAD_TO_DEG;

    return new BallisticSolution(vTotalNew, thetaDeg, deltaFloorAngleDeg);
  }

  /**
   * Calculates the predicted landing position of the projectile after time T, accounting for 3D
   * drag (horizontal + vertical components). * @param vxLaunch Initial horizontal X velocity (m/s)
   *
   * @param vyLaunch Initial horizontal Y velocity (m/s)
   * @param vzLaunch Initial vertical velocity (m/s)
   * @return double[] {finalX, finalY, finalZ}
   */
  public static double[] predictLandingPose(
      double xStart, double yStart, double vxLaunch, double vyLaunch, double vzLaunch) {
    final int RK_STEPS = 25;
    final double dt = T / RK_STEPS;
    final double G = 9.806;

    double x = xStart; // Start from where the robot will be at release
    double y = yStart;
    double z = 0.0;

    double vx = vxLaunch;
    double vy = vyLaunch;
    double vz = vzLaunch;

    for (int i = 0; i < RK_STEPS; i++) {
      double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

      // Midpoint accelerations
      double ax = -ALPHA * speed * vx;
      double ay = -ALPHA * speed * vy;
      double az = -ALPHA * speed * vz - G;

      // RK2 Midpoint velocity
      double vxMid = vx + 0.5 * dt * ax;
      double vyMid = vy + 0.5 * dt * ay;
      double vzMid = vz + 0.5 * dt * az;
      double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);

      // Update state
      x += dt * vxMid;
      y += dt * vyMid;
      z += dt * vzMid;

      vx += dt * (-ALPHA * speedMid * vxMid);
      vy += dt * (-ALPHA * speedMid * vyMid);
      vz += dt * (-ALPHA * speedMid * vzMid - G);
    }
    return new double[] {x, y, z};
  }

  public static double restingAngularVelocity = ShootingConstants.FREE_ANGULAR_VELOCITY;
  public static double RADIUS = ShootingConstants.SHOOTER_RADIUS;
  public static double E = ShootingConstants.SHOOTER_EFFICIENCY;

  /* =======================
   * Output State Variables
   * ======================= */

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
  public static double computeMotorAdjustment(double launchSpeed) {

    long start = System.nanoTime();

    desiredAngularVelocity = launchSpeed / (E * RADIUS) / 2 / Math.PI;
    long end = System.nanoTime();
    return desiredAngularVelocity;
    // TODO: Log start/end times
  }

}
