package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.ShootingUtil;
import frc.robot.util.ShootingUtil.BallisticSolution;
import org.littletonrobotics.junction.Logger;

public class ShootingPredictions {

  /**
   * This is the instance of {@code ShootingPredictions} that should be referenced by other shooter
   * classes.
   */
  private static final ShootingPredictions calculator = new ShootingPredictions();

  private BallisticSolution lastRawSolution = new BallisticSolution(0, 0, 0);

  public static ShootingPredictions getCalculator() {
    return calculator;
  }

  /**
   * Vertical (elevation) angle to set the shooter to, in degrees. Updated by {@link
   * #updateShootingParameters} after ballistic calculation.
   */
  private double verticalShootingAngle = 0;

  /**
   * Horizontal offset angle (floor-plane) to aim the shooter, in degrees. Accounts for lateral
   * offsets between robot heading and target after predicting motion during reload.
   */
  private double horizontalOffsetShootingAngle = 0;

  /**
   * Total horizontal shooting angle (floor-plane) in degrees, combining the robot's current heading
   * and the required offset to aim at the target. This is the absolute angle the shooter should be
   * set to in the floor frame.
   */
  private double horizontalTotalShootingAngle = 0;

  /**
   * Computed launch velocity (m/s) required to reach the target using the current predicted range
   * and offsets. Obtained from {@code FastBallisticCalculator}.
   */
  private double launchVelocity = 0;

  /**
   * Desired angular velocity of the shooter wheel after adjustment (rad/s) - target speed
   * post-launch. Computed by {@code MotorAdjustmentCalculator}.
   */
  private double desiredAngularVelocity = 0;

  /**
   * Most recently computed field-relative angle to hub (radians). Stored so updateAdvantageScope
   * can reconstruct the absolute turret azimuth.
   */
  private double lastFieldRelativeAngleToHub = 0;

  /**
   * Most recently computed turret position in the field frame. Stored so updateAdvantageScope can
   * use the same origin as the solver.
   */
  private Translation2d lastTurretPos = Translation2d.kZero;

  private static double rateLimit(double current, double target) {
    double delta = MathUtil.inputModulus(target - current, -180.0, 180.0);
    return current + delta;
  }

  /**
   * Computes turret geometry from the current robot state and updates all shooting parameters. Must
   * be called before {@link #updateAdvantageScope} each cycle.
   *
   * @param robotPose current robot pose
   * @param chassisSpeeds current robot chassis speeds (robot-relative)
   * @param acceleration current robot acceleration (robot-relative, as Transform2d)
   */
  public void updateDistanceAndAngle(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Transform2d acceleration) {

    // Offset robot center to turret center in field frame
    Translation2d turretOffset =
        Constants.ROBOT_TO_TURRET
            .plus(new Transform2d(0, 0, robotPose.getRotation()))
            .getTranslation();
    Translation2d turretPos = robotPose.getTranslation().plus(turretOffset);

    var targetPose = ShootingUtil.getShootingTarget(robotPose);
    Translation2d hub = targetPose.getTranslation();

    double dx = hub.getX() - turretPos.getX();
    double dy = hub.getY() - turretPos.getY();
    double distanceToHub = Math.hypot(dx, dy);
    double fieldRelativeAngleToHub = Math.atan2(dy, dx);

    // Store for use by updateAdvantageScope
    lastFieldRelativeAngleToHub = fieldRelativeAngleToHub;
    lastTurretPos = turretPos;

    updateShootingParameters(
        distanceToHub, fieldRelativeAngleToHub, chassisSpeeds, acceleration, robotPose);
  }

  /**
   * Computes and logs AdvantageScope visualization data for the predicted ball trajectory. Must be
   * called after {@link #updateDistanceAndAngle} each cycle.
   *
   * @param robotPose current robot pose
   * @param chassisSpeeds current robot chassis speeds (robot-relative)
   * @param acceleration current robot acceleration (robot-relative, as Transform2d)
   */
  public void updateAdvantageScope(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Transform2d acceleration) {

    // Convert to field-frame velocity and acceleration
    Rotation2d robotRotation = robotPose.getRotation();
    double cosR = Math.cos(robotRotation.getRadians());
    double sinR = Math.sin(robotRotation.getRadians());

    double vxField =
        chassisSpeeds.vxMetersPerSecond * cosR - chassisSpeeds.vyMetersPerSecond * sinR;
    double vyField =
        chassisSpeeds.vxMetersPerSecond * sinR + chassisSpeeds.vyMetersPerSecond * cosR;

    double axField = acceleration.getX() * cosR - acceleration.getY() * sinR;
    double ayField = acceleration.getX() * sinR + acceleration.getY() * cosR;

    // Predict turret position at release (after reload time R)
    double reloadTime = Constants.ShootingConstants.RELOAD_TIME;
    double turretRelX =
        lastTurretPos.getX() + vxField * reloadTime + 0.5 * axField * reloadTime * reloadTime;
    double turretRelY =
        lastTurretPos.getY() + vyField * reloadTime + 0.5 * ayField * reloadTime * reloadTime;

    Pose2d turretPose = new Pose2d(lastTurretPos, robotPose.getRotation());
    Pose2d releasePose = new Pose2d(turretRelX, turretRelY, robotPose.getRotation());

    // Robot velocity at release (field frame)
    double vxRobotRelease = vxField + axField * reloadTime;
    double vyRobotRelease = vyField + ayField * reloadTime;

    // Use raw (un-rate-limited) solver output for trajectory visualization
    double hoodRad = Math.toRadians(lastRawSolution.hoodAngleDeg());
    double turretFieldRad =
        Math.toRadians(lastRawSolution.deltaAzimuthDeg()) + lastFieldRelativeAngleToHub;
    double launchSpeed = lastRawSolution.launchSpeed();

    // Shooter-frame floor velocity, then add robot release velocity for ground frame
    double vFloorShooter = launchSpeed * Math.cos(hoodRad);
    double vxLaunch = vFloorShooter * Math.cos(turretFieldRad) + vxRobotRelease;
    double vyLaunch = vFloorShooter * Math.sin(turretFieldRad) + vyRobotRelease;
    double vzLaunch = launchSpeed * Math.sin(hoodRad);

    double[] landing =
        ShootingUtil.predictLandingPose(turretRelX, turretRelY, vxLaunch, vyLaunch, vzLaunch);

    Logger.recordOutput("Shooter/TurretPose", turretPose);
    Logger.recordOutput("Shooter/ReleasePose", releasePose);
    Logger.recordOutput(
        "Shooter/PredictedLandingPose", new Pose2d(landing[0], landing[1], Rotation2d.kZero));
    Logger.recordOutput(
        "Shooter/TrajectoryLine",
        new Pose2d[] {releasePose, new Pose2d(landing[0], landing[1], Rotation2d.kZero)});
    Logger.recordOutput(
        "Shooter/DistanceToHub",
        Math.hypot(
            ShootingUtil.getShootingTarget(robotPose).getTranslation().getX()
                - lastTurretPos.getX(),
            ShootingUtil.getShootingTarget(robotPose).getTranslation().getY()
                - lastTurretPos.getY()));
    Logger.recordOutput("Shooter/TurretPos", lastTurretPos.toString());
    Logger.recordOutput("Shooter/RotationsPerSecond", desiredAngularVelocity);
  }

  /**
   * Update all shooting-related parameters using the current robot motion and target geometry.
   *
   * @param distanceToHub current straight-line distance (meters) to the hub
   * @param fieldRelativeAngleToHub current field-relative angle (radians) to the hub
   * @param chassisSpeeds current robot chassis speeds (m/s)
   */
  public void updateShootingParameters(
      double distanceToHub,
      double fieldRelativeAngleToHub,
      ChassisSpeeds chassisSpeeds,
      Pose2d robotPose) {
    updateShootingParameters(
        distanceToHub, fieldRelativeAngleToHub, chassisSpeeds, new Transform2d(), robotPose);
  }

  public void updateShootingParameters(
      double distanceToHub,
      double fieldRelativeAngleToHub,
      ChassisSpeeds chassisSpeeds,
      Transform2d acceleration,
      Pose2d robotPose) {

    Rotation2d robotRotation = robotPose.getRotation();
    double cos = Math.cos(robotRotation.getRadians());
    double sin = Math.sin(robotRotation.getRadians());

    double vxField = chassisSpeeds.vxMetersPerSecond * cos - chassisSpeeds.vyMetersPerSecond * sin;
    double vyField = chassisSpeeds.vxMetersPerSecond * sin + chassisSpeeds.vyMetersPerSecond * cos;

    double axField = acceleration.getX() * cos - acceleration.getY() * sin;
    double ayField = acceleration.getX() * sin + acceleration.getY() * cos;

    BallisticSolution calculation =
        ShootingUtil.computeBallistics(
            distanceToHub,
            Math.toDegrees(fieldRelativeAngleToHub),
            vxField,
            vyField,
            axField,
            ayField);

    lastRawSolution = calculation;

    double newVertical = calculation.hoodAngleDeg();
    double newOffset = calculation.deltaAzimuthDeg();
    double newTotal = newOffset + Math.toDegrees(fieldRelativeAngleToHub);
    double newLaunchVelocity = calculation.launchSpeed();

    verticalShootingAngle = rateLimit(verticalShootingAngle, newVertical);
    horizontalTotalShootingAngle = rateLimit(horizontalTotalShootingAngle, newTotal);
    launchVelocity = rateLimit(launchVelocity, newLaunchVelocity);

    horizontalOffsetShootingAngle =
        horizontalTotalShootingAngle - Math.toDegrees(fieldRelativeAngleToHub);
    desiredAngularVelocity = ShootingUtil.computeMotorAdjustment(calculation.launchSpeed());

    Logger.recordOutput("Shooting/VerticalShootingAngle", verticalShootingAngle);
    Logger.recordOutput("Shooting/HorizontalOffsetShootingAngle", horizontalOffsetShootingAngle);
    Logger.recordOutput("Shooting/HorizontalTotalShootingAngle", horizontalTotalShootingAngle);
    Logger.recordOutput("Shooting/LaunchVelocity", launchVelocity);
  }

  public double getVerticalShootingAngle() {
    return verticalShootingAngle;
  }

  public double getHorizontalOffsetShootingAngle() {
    return horizontalOffsetShootingAngle;
  }

  public double getLaunchVelocity() {
    return launchVelocity;
  }

  public double getDesiredAngularVelocity() {
    return desiredAngularVelocity;
  }

  public double getHorizontalTotalShootingAngle() {
    return horizontalTotalShootingAngle;
  }

  public BallisticSolution getLastRawSolution() {
    return lastRawSolution;
  }
}
