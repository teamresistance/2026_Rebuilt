package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;

public class TurretConfidenceUtil {

  private static final double ERROR_STD_DEV = 2.5;

  // Portion of rotation error that remains after turret compensation
  // 0 = perfect turret
  // 1 = no compensation (stationary shooter)
  private static final double ROTATION_COMPENSATION_FACTOR = 0.3;

  private static final double DISTANCE_FACTOR = 0.3;

  public static double calculateConfidence(SwerveDriveIO drive) {
    // Get current robot pose and velocity
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeedsFieldRelative();

    // Update target position based on current robot pose
    Translation2d targetPosition =
        ShootingUtil.getShootingTarget(drive.getPose(), false).getTranslation();

    // Distance to target calculated with field-relative coordinates
    double distance = pose.getTranslation().getDistance(targetPosition);

    // Time of flight calculation based on distance and current velocity
    double timeOfFlight =
        ShootingConstants.getTimeOfFlight(
            ShootingUtil.getVirtualDistanceToTarget(pose, speeds, false));

    // Calculate lateral velocity of the robot with the x and y velocity
    double lateralVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // Translation drift during flight (how far the robot will move laterally while the shot is in
    // the air)
    double translationError = lateralVelocity * timeOfFlight;

    // Rotational drift arc displacement at target distance
    double rawRotationError = speeds.omegaRadiansPerSecond * timeOfFlight * distance;

    // Apply turret compensation
    double rotationError = rawRotationError * ROTATION_COMPENSATION_FACTOR;

    double distanceError = distance * DISTANCE_FACTOR;

    // Combine errors
    double totalError =
        Math.cbrt(
            translationError * translationError
                + rotationError * rotationError
                + distanceError * distanceError);

    // Convert miss distance to probability (Gaussian curve equation)
    double probability = Math.exp(-Math.pow(totalError, 2) / (2 * Math.pow(ERROR_STD_DEV, 2)));

    // Scale to percentage for easier interpretation
    return Math.round(probability * 100);
  }
}
