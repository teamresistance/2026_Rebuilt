package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;

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
        ShootingConstants.getTimeOfFlight(ShootingUtil.getVirtualDistanceToTarget(pose, speeds));

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
}
