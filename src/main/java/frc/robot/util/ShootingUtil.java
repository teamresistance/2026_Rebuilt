package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShootingMaps;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingUtil {

  private ShootingUtil() {}

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the angle converted to an equal amount of motor revolutions. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodRevs(double degrees) {
    return degrees * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of yaw.
   */
  public static double toTurretDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  /**
   * Returns the angle converted to an equal amount of motor revolutions. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of yaw.
   */
  public static double toTurretRevs(double degrees) {
    return degrees * Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  /**
   * Returns the angle from the predicted shooting position to the goal center. Goal is determined
   * by {@code FieldConstants.getShootingTarget()}
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot velocity
   * @param estimatedAirtime estimated airtime of the projectile (seconds)
   * @return the angle to aim to the turret at, in degrees
   */
  public static double getAngleToAim(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, double estimatedAirtime) {

    Pose2d goalPose = getShootingTarget(robotPose);
    Logger.recordOutput("Shooter/GoalPose", goalPose);

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

    Translation2d futureTurretPos = turretPos.plus(fieldVelocity.times(estimatedAirtime));

    Translation2d toTarget = goalPose.getTranslation().minus(futureTurretPos);
    double turretAngleField = Math.atan2(toTarget.getY(), toTarget.getX());
    double turretAngleRobotRelative =
        MathUtil.angleModulus(turretAngleField - robotPose.getRotation().getRadians());

    Logger.recordOutput(
        "Shooter/Calculated Virtual Pose",
        new Pose2d(futureTurretPos.getX(), futureTurretPos.getY(), Rotation2d.kZero));

    return Math.toDegrees(turretAngleRobotRelative);
  }

  /**
   * Gets the distance from the "virtual turret pose" to the goal. Goal is determined by {@code
   * FieldConstants.getShootingTarget()}
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot chassis speeds, field-relative
   * @return the distance between the hub and the virtual turret pose
   */
  public static double getVirtualDistanceToTarget(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

    Pose2d goalPose = getShootingTarget(robotPose);
    Logger.recordOutput("Shooter/GoalPose", goalPose);

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

    Translation2d futureTurretPos;
    double lookaheadDistance = turretPos.getDistance(goalPose.getTranslation());

    for (int i = 0; i < 20; i++) {
      double timeOfFlight = ShootingMaps.getTimeOfFlight(lookaheadDistance);
      futureTurretPos = turretPos.plus(fieldVelocity.times(timeOfFlight));
      lookaheadDistance = futureTurretPos.getDistance(goalPose.getTranslation());
    }

    return lookaheadDistance;
  }

  public static Pose2d getShootingTarget(Pose2d pose) {
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      return Pose2d.kZero;
    }

    boolean isTop = pose.getY() >= FieldConstants.TOP_BOTTOM_SPLIT_Y;

    if (allianceOpt.get() == DriverStation.Alliance.Blue) {
      if (pose.getX() <= FieldConstants.BLUE_SHOOTING_ZONE_END) {
        return FieldConstants.BLUE_GOAL_CENTER;
      }
      if (pose.getX() > FieldConstants.NEUTRAL_ZONE_BLUESIDE) {
        return isTop
            ? FieldConstants.BLUE_TOP_FERRY_TARGET
            : FieldConstants.BLUE_BOTTOM_FERRY_TARGET;
      }
    } else {
      if (pose.getX() >= FieldConstants.RED_SHOOTING_ZONE_START) {
        return FieldConstants.RED_GOAL_CENTER;
      }
      if (pose.getX() < FieldConstants.NEUTRAL_ZONE_REDSIDE) {
        return isTop ? FieldConstants.RED_TOP_FERRY_TARGET : FieldConstants.RED_BOTTOM_FERRY_TARGET;
      }
    }

    return Pose2d.kZero;
  }

  /** Returns a 0 if shooting to hub and a 1 if ferrying. */
  public static int getShootingType(Supplier<Pose2d> poseSupplier) {
    Pose2d pose = poseSupplier.get();
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      return 0;
    }

    if (allianceOpt.get() == DriverStation.Alliance.Blue
        && pose.getX() <= FieldConstants.BLUE_SHOOTING_ZONE_END) {
      return 0;
    }
    if (allianceOpt.get() == DriverStation.Alliance.Red
        && pose.getX() >= FieldConstants.RED_SHOOTING_ZONE_START) {
      return 0;
    }
    return 1;
  }

  // ------------------------------------Calculator methods------------------------------------

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
  private static final double INV_T = 1.0 / T;

  /** Alpha constant for RK2 correction - derived from drag coefficient and mass */
  private static final double ALPHA = K / M;

  private static final double HUB_HEIGHT = ShootingConstants.HUB_HEIGHT;

  private static final int RK_STEPS = 30; // Increased for sub-centimeter precision
  private static final int SOLVE_ITERS = 3;
  private static final double DT = T / RK_STEPS;
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
   * @param d distance to the target along the floor (meters)
   * @param floorAzimuthDeg absolute azimuth angle of the target in the floor frame (degrees)
   * @param vX robot floor velocity in the X direction (m/s)
   * @param vY robot floor velocity in the Y direction (m/s)
   */
  public static BallisticSolution computeBallistics(
      double d, double floorAzimuthDeg, double vX, double vY) {
    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // 1. Hub position is fixed in the Field Frame
    double xHub = d * Math.cos(azRad);
    double yHub = d * Math.sin(azRad);

    // 2. ANALYTIC BASE (Initial guess to hit a stationary target distance d)
    double vFloorField = (M / (K * T)) * (Math.exp((K * d) / M) - 1.0);
    double vxGuess = vFloorField * Math.cos(azRad);
    double vyGuess = vFloorField * Math.sin(azRad);
    double vzGuess = (VY + 0.5 * 9.806 * T); // Initial vertical guess to counter gravity

    for (int iter = 0; iter < SOLVE_ITERS; iter++) {
      double x = 0.0, y = 0.0, z = 0.0;
      double vx = vxGuess, vy = vyGuess, vz = vzGuess;

      for (int i = 0; i < RK_STEPS; i++) {
        // Instantaneous 3D speed (The "True" Drag source)
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double dragFactor = -ALPHA * speed;

        // RK2 Midpoint
        double vxMid = vx + 0.5 * DT * (dragFactor * vx);
        double vyMid = vy + 0.5 * DT * (dragFactor * vy);
        double vzMid = vz + 0.5 * DT * (dragFactor * vz - G);

        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);
        double dragFactorMid = -ALPHA * speedMid;

        // State Update
        x += DT * vxMid;
        y += DT * vyMid;
        z += DT * vzMid;

        vx += DT * (dragFactorMid * vxMid);
        vy += DT * (dragFactorMid * vyMid);
        vz += DT * (dragFactorMid * vzMid - G);
      }

      // Compute 3D Error
      double errX = x - xHub;
      double errY = y - yHub;
      double errZ = z - HUB_HEIGHT;

      // Newton correction (using INV_T for horizontal,
      // vertical requires less gain due to gravity's dominance)
      vxGuess -= errX * INV_T;
      vyGuess -= errY * INV_T;
      vzGuess -= errZ * INV_T;
    }

    // 4. GALILEAN TRANSFORMATION (The "Magic" Step)
    // vxGuess is what the ball needs to be doing relative to the ground.
    // The shooter only provides the difference between that and the robot's velocity.
    double vxShooter = vxGuess - vX;
    double vyShooter = vyGuess - vY;

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
   * @param d distance to the target along the floor (meters)
   * @param floorAzimuthDeg absolute azimuth angle of the target in the floor frame (degrees)
   * @param vX robot floor velocity in the X direction (m/s)
   * @param vY robot floor velocity in the Y direction (m/s)
   * @param aX robot floor acceleration in the X direction (m/s²)
   * @param aY robot floor acceleration in the Y direction (m/s²)
   */
  public static BallisticSolution computeBallistics(
      double d, double floorAzimuthDeg, double vX, double vY, double aX, double aY) {
    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // 1. PROJECT ROBOT AND TARGET RELATIVE TO RELEASE POINT
    // Robot displacement from now until release (t=R)
    double xRel = vX * R + 0.5 * aX * R * R;
    double yRel = vY * R + 0.5 * aY * R * R;

    // Robot velocity at release
    double vxRobotRelease = vX + aX * R;
    double vyRobotRelease = vY + aY * R;

    // Hub position relative to the START position (field frame)
    double xHubField = d * Math.cos(azRad);
    double yHubField = d * Math.sin(azRad);

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
        double vxMid = vx + 0.5 * DT * (dragFactor * vx);
        double vyMid = vy + 0.5 * DT * (dragFactor * vy);
        double vzMid = vz + 0.5 * DT * (dragFactor * vz - G);

        double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);
        double dragMid = -ALPHA * speedMid;

        x += DT * vxMid;
        y += DT * vyMid;
        z += DT * vzMid;
        vx += DT * (dragMid * vxMid);
        vy += DT * (dragMid * vyMid);
        vz += DT * (dragMid * vzMid - G);
      }

      // Error relative to the hub's position from the release point
      double errX = x - dxTarget;
      double errY = y - dyTarget;
      double errZ = z - HUB_HEIGHT;

      // Apply corrections
      vxGuess -= errX * INV_T;
      vyGuess -= errY * INV_T;
      vzGuess -= errZ * INV_T;
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
    final double DT = T / RK_STEPS;
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
      double vxMid = vx + 0.5 * DT * ax;
      double vyMid = vy + 0.5 * DT * ay;
      double vzMid = vz + 0.5 * DT * az;
      double speedMid = Math.sqrt(vxMid * vxMid + vyMid * vyMid + vzMid * vzMid);

      // Update state
      x += DT * vxMid;
      y += DT * vyMid;
      z += DT * vzMid;

      vx += DT * (-ALPHA * speedMid * vxMid);
      vy += DT * (-ALPHA * speedMid * vyMid);
      vz += DT * (-ALPHA * speedMid * vzMid - G);
    }
    return new double[] {x, y, z};
  }

  public static final double RADIUS = ShootingConstants.SHOOTER_RADIUS;
  public static final double E = ShootingConstants.SHOOTER_EFFICIENCY;

  /** Computes the required angular velocity to launch the fuel at the desired launch speed. */
  public static double computeMotorAdjustment(double launchSpeed) {
    return launchSpeed / (E * RADIUS) / 2 / Math.PI;
  }
}
