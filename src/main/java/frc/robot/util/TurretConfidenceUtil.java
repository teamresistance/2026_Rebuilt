package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingMaps;
import frc.robot.subsystems.shooter.ShootingPredictions;

public class TurretConfidenceUtil {

  // TODO: Tune the standard deviation to the robot's accuracy
  private static final double ERROR_STD_DEV = 1.0;

  public static double calculateConfidence(SwerveDriveIO drive) {
    // Get current robot pose and velocity
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeedsFieldRelative();

    // Update target position based on current robot pose
    Translation2d targetPosition = ShootingUtil.getShootingTarget(drive.getPose()).getTranslation();

    // Distance to target calculated with field-relative coordinates
    double distance = pose.getTranslation().getDistance(targetPosition);

    // Time of flight calculation based on distance and current velocity
    double timeOfFlight =
        ShootingMaps.getTimeOfFlight(ShootingUtil.getVirtualDistanceToTarget(pose, speeds));

    // Calculate lateral velocity of the robot with the x and y velocity
    double lateralVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // Translation drift during flight
    double translationError = lateralVelocity * timeOfFlight;

    // Rotational drift arc displacement at target distance
    double rotationError = speeds.omegaRadiansPerSecond * timeOfFlight * distance;

    // Combine errors
    double totalError =
        Math.sqrt(translationError * translationError + rotationError * rotationError);

    // Convert miss distance to probability (Gaussian curve equation)
    double probability = Math.exp(-Math.pow(totalError, 2) / (2 * Math.pow(ERROR_STD_DEV, 2)));

    // Scale to percentage for easier interpretation
    return probability * 100;
  }

  /**
   * Calculates shooting confidence (0-100) for the CALC workflow.
   *
   * <p>Differences from the MAPS version:
   *
   * <ul>
   *   <li>Time of flight comes from the fixed ballistic flight time constant rather than a lookup
   *       table, since the solver targets a fixed T.
   *   <li>The error is evaluated at the predicted release position after reload time, not the
   *       current robot position, because the robot moves before the ball is released.
   *   <li>Velocity at release is computed using current acceleration, so a decelerating robot
   *       correctly produces lower error than one still at full speed.
   *   <li>An additional RPM readiness error term penalizes the confidence if the flywheel has not
   *       yet reached its target speed, since muzzle velocity error maps directly to position
   *       error.
   * </ul>
   *
   * @param drive the swerve drive subsystem
   * @return confidence as a percentage [0, 100]
   */
  public static double calculateConfidenceCalc(SwerveDriveIO drive, ShooterIO shooter) {
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeedsFieldRelative();

    double timeOfFlight = ShootingConstants.MINIMUM_TIME_OF_FLIGHT;
    double reloadTime = ShootingConstants.RELOAD_TIME;

    // Rotate robot-relative acceleration into field frame
    double cosR = Math.cos(pose.getRotation().getRadians());
    double sinR = Math.sin(pose.getRotation().getRadians());
    double axField = drive.getAcceleration().getX() * cosR - drive.getAcceleration().getY() * sinR;
    double ayField = drive.getAcceleration().getX() * sinR + drive.getAcceleration().getY() * cosR;

    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;

    // Velocity at ball release after reload time
    double vxRelease = vx + axField * reloadTime;
    double vyRelease = vy + ayField * reloadTime;

    // Predicted release position
    double releaseX = pose.getX() + vx * reloadTime + 0.5 * axField * reloadTime * reloadTime;
    double releaseY = pose.getY() + vy * reloadTime + 0.5 * ayField * reloadTime * reloadTime;

    // Distance from predicted release position to target
    Translation2d targetPosition = ShootingUtil.getShootingTarget(pose).getTranslation();
    double distanceAtRelease =
        Math.hypot(targetPosition.getX() - releaseX, targetPosition.getY() - releaseY);

    // Translation error: lateral speed at release drifts the ball during flight
    double lateralVelocityAtRelease = Math.hypot(vxRelease, vyRelease);
    double translationError = lateralVelocityAtRelease * timeOfFlight;

    // Rotation error: angular velocity sweeps the aim point across the target distance
    double rotationError = speeds.omegaRadiansPerSecond * timeOfFlight * distanceAtRelease;

    // RPM readiness error: flywheel not at speed means muzzle velocity is off,
    // which maps to a position error proportional to fractional speed deficit * distance
    double targetRPS = ShootingPredictions.getCalculator().getDesiredAngularVelocity();
    double rpsError =
        (targetRPS > 0)
            ? Math.abs(shooter.getFlywheelRPS() - targetRPS) / targetRPS * distanceAtRelease
            : 0.0;

    double totalError =
        Math.sqrt(
            translationError * translationError
                + rotationError * rotationError
                + rpsError * rpsError);

    double probability = Math.exp(-Math.pow(totalError, 2) / (2 * Math.pow(ERROR_STD_DEV, 2)));

    return probability * 100;
  }
}
