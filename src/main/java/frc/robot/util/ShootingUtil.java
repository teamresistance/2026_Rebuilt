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
  private static final double VZ = ShootingConstants.VERTICAL_VELOCITY_COMPONENT;

  /** Mass coefficient (kg) - affects drag acceleration calculations */
  public static final double M = ShootingConstants.FUEL_MASS;

  /** Drag coefficient - exponential factor in air resistance equation */
  private static final double K = ShootingConstants.QUADRATIC_DRAG_COEFFICIENT;

  /** Fixed flight time (seconds) - predetermined projectile flight duration */
  private static final double T = ShootingConstants.MINIMUM_TIME_OF_FLIGHT;

  /** Reload time (seconds) - used for ballistic calculations with acceleration */
  private static final double R = ShootingConstants.RELOAD_TIME;

  /** Inverse of flight time (1/T) - precomputed for efficiency in calculations */
  private static final double GAIN = 1.0 / ((M / K) * (1.0 - Math.exp(-K * T / M)));

  /** Alpha constant for RK2 correction - derived from drag coefficient and mass */
  private static final double ALPHA = K / M;

  private static final double DESIRED_HEIGHT =
      ShootingConstants.HUB_HEIGHT - ShootingConstants.TURRET_HEIGHT;

  private static final int RK_STEPS = 30; // Increased for sub-centimeter precision
  private static final int SOLVE_ITERS = 5;
  private static final double DT = T / RK_STEPS;
  private static final double G = 9.806;

  public record BallisticSolution(
      double launchSpeed, double hoodAngleDeg, double deltaAzimuthDeg) {}

  /**
   * Integrates the 3D drag trajectory for exactly T seconds from the given initial velocity,
   * returning final {x, y, z, vx, vy, vz}.
   */
  private static double[] integrateTrajectory(
      double xStart, double yStart, double vx0, double vy0, double vz0) {

    double x = xStart;
    double y = yStart;
    double z = 0.0;
    double vx = vx0;
    double vy = vy0;
    double vz = vz0;

    for (int i = 0; i < RK_STEPS; i++) {
      double s1 = Math.sqrt(vx * vx + vy * vy + vz * vz);
      double d1 = -ALPHA * s1;
      double k1vx = d1 * vx;
      double k1vy = d1 * vy;
      double k1vz = d1 * vz - G;

      double vx2 = vx + 0.5 * DT * k1vx;
      double vy2 = vy + 0.5 * DT * k1vy;
      double vz2 = vz + 0.5 * DT * k1vz;
      double s2 = Math.sqrt(vx2 * vx2 + vy2 * vy2 + vz2 * vz2);
      double d2 = -ALPHA * s2;
      double k2vx = d2 * vx2;
      double k2vy = d2 * vy2;
      double k2vz = d2 * vz2 - G;

      double vx3 = vx + 0.5 * DT * k2vx;
      double vy3 = vy + 0.5 * DT * k2vy;
      double vz3 = vz + 0.5 * DT * k2vz;
      double s3 = Math.sqrt(vx3 * vx3 + vy3 * vy3 + vz3 * vz3);
      double d3 = -ALPHA * s3;
      double k3vx = d3 * vx3;
      double k3vy = d3 * vy3;
      double k3vz = d3 * vz3 - G;

      double vx4 = vx + DT * k3vx;
      double vy4 = vy + DT * k3vy;
      double vz4 = vz + DT * k3vz;
      double s4 = Math.sqrt(vx4 * vx4 + vy4 * vy4 + vz4 * vz4);
      double d4 = -ALPHA * s4;
      double k4vx = d4 * vx4;
      double k4vy = d4 * vy4;
      double k4vz = d4 * vz4 - G;

      x += (DT / 6.0) * (vx + 2 * vx2 + 2 * vx3 + vx4);
      y += (DT / 6.0) * (vy + 2 * vy2 + 2 * vy3 + vy4);
      z += (DT / 6.0) * (vz + 2 * vz2 + 2 * vz3 + vz4);
      vx += (DT / 6.0) * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
      vy += (DT / 6.0) * (k1vy + 2 * k2vy + 2 * k3vy + k4vy);
      vz += (DT / 6.0) * (k1vz + 2 * k2vz + 2 * k3vz + k4vz);
    }
    return new double[] {x, y, z, vx, vy, vz};
  }

  /**
   * Runs the RK4 Newton solve loop, iterating launch velocity components toward the given
   * horizontal and vertical targets. Corrects vertical first each iteration to minimize
   * horizontal-vertical drag coupling error.
   *
   * @param vxGuess initial X velocity guess (m/s), field frame
   * @param vyGuess initial Y velocity guess (m/s), field frame
   * @param vzGuess initial Z velocity guess (m/s)
   * @param xTarget horizontal X target position (m), relative to launch origin
   * @param yTarget horizontal Y target position (m), relative to launch origin
   * @return double[] {vxSolved, vySolved, vzSolved}
   */
  private static double[] solveVelocity(
      double vxGuess, double vyGuess, double vzGuess, double xTarget, double yTarget) {

    for (int iter = 0; iter < SOLVE_ITERS; iter++) {
      double[] result = integrateTrajectory(0.0, 0.0, vxGuess, vyGuess, vzGuess);

      double errX = result[0] - xTarget;
      double errY = result[1] - yTarget;
      double errZ = result[2] - DESIRED_HEIGHT;

      vzGuess -= errZ * GAIN;
      vxGuess -= errX * GAIN;
      vyGuess -= errY * GAIN;
    }

    return new double[] {vxGuess, vyGuess, vzGuess};
  }

  public static BallisticSolution computeBallistics(
      double d, double floorAzimuthDeg, double vX, double vY) {
    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    // Project robot forward to release point (no acceleration, constant velocity)
    double xRel = vX * R;
    double yRel = vY * R;
    // Velocity at release is unchanged (zero acceleration)
    double vxRobotRelease = vX;
    double vyRobotRelease = vY;

    // Hub position relative to the release point
    double xHubField = d * Math.cos(azRad);
    double yHubField = d * Math.sin(azRad);
    double dxTarget = xHubField - xRel;
    double dyTarget = yHubField - yRel;
    double dRelative = Math.sqrt(dxTarget * dxTarget + dyTarget * dyTarget);

    double vFloorField = (M / (K * T)) * (Math.exp((K * dRelative) / M) - 1.0);
    double vxGuess = vFloorField * (dxTarget / dRelative);
    double vyGuess = vFloorField * (dyTarget / dRelative);
    double vzGuess = VZ;

    double[] solved = solveVelocity(vxGuess, vyGuess, vzGuess, dxTarget, dyTarget);
    double vxSolved = solved[0];
    double vySolved = solved[1];
    double vzSolved = solved[2];

    double vxShooter = vxSolved - vxRobotRelease;
    double vyShooter = vySolved - vyRobotRelease;
    double vFloorCorrected = Math.sqrt(vxShooter * vxShooter + vyShooter * vyShooter);
    double vTotalNew = Math.sqrt(vFloorCorrected * vFloorCorrected + vzSolved * vzSolved);
    double thetaDeg = Math.atan2(vzSolved, vFloorCorrected) * ShootingConstants.RAD_TO_DEG;
    double deltaFloorAngleDeg =
        (Math.atan2(vyShooter, vxShooter) - azRad) * ShootingConstants.RAD_TO_DEG;

    return new BallisticSolution(vTotalNew, thetaDeg, deltaFloorAngleDeg);
  }

  public static BallisticSolution computeBallistics(
      double d, double floorAzimuthDeg, double vX, double vY, double aX, double aY) {
    double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;

    double xRel = vX * R + 0.5 * aX * R * R;
    double yRel = vY * R + 0.5 * aY * R * R;
    double vxRobotRelease = vX + aX * R;
    double vyRobotRelease = vY + aY * R;

    double xHubField = d * Math.cos(azRad);
    double yHubField = d * Math.sin(azRad);
    double dxTarget = xHubField - xRel;
    double dyTarget = yHubField - yRel;
    double dRelative = Math.sqrt(dxTarget * dxTarget + dyTarget * dyTarget);

    double vFloorGuess = (M / (K * T)) * (Math.exp((K * dRelative) / M) - 1.0);
    double vxGuess = vFloorGuess * (dxTarget / dRelative);
    double vyGuess = vFloorGuess * (dyTarget / dRelative);
    double vzGuess = VZ;

    double[] solved = solveVelocity(vxGuess, vyGuess, vzGuess, dxTarget, dyTarget);
    double vxSolved = solved[0];
    double vySolved = solved[1];
    double vzSolved = solved[2];

    double vxShooter = vxSolved - vxRobotRelease;
    double vyShooter = vySolved - vyRobotRelease;
    double vFloorFinal = Math.sqrt(vxShooter * vxShooter + vyShooter * vyShooter);
    double vTotalNew = Math.sqrt(vFloorFinal * vFloorFinal + vzSolved * vzSolved);
    double thetaDeg = Math.atan2(vzSolved, vFloorFinal) * ShootingConstants.RAD_TO_DEG;
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
    double[] result = integrateTrajectory(xStart, yStart, vxLaunch, vyLaunch, vzLaunch);
    return new double[] {result[0], result[1], result[2]};
  }

  public static final double RADIUS = ShootingConstants.SHOOTER_RADIUS;
  public static final double E = ShootingConstants.SHOOTER_EFFICIENCY;

  /**
   * Computes the required angular velocity in RPS to launch the fuel at the desired launch speed.
   */
  public static double computeMotorAdjustment(double launchSpeed) {
    System.out.println("Tangential velocity:" + 1.4 * launchSpeed);
    return E * launchSpeed / RADIUS / (2 * Math.PI);
  }
}
