package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;

public class TurretConfidenceUtil {

  private SwerveDriveIO drive; // Default to real drive, can be overridden for testing

  public void instantiateDrive(SwerveDriveIO drive) {
    this.drive = drive; // instantiate the drive subsystem for use in confidence calculations
  }

  // TODO: Tune the standard deviation to the robot's accuracy
  private final double ERROR_STD_DEV = 1.0;

  // Call this in Robot.java to update confidence
  public double calculateConfidence() {

    // Get current robot pose and velocity
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeeds();

    // Update target position based on current robot pose
    Translation2d TARGET_POSITION =
        ShootingUtil.getShootingTarget(drive.getPose()).getTranslation();

    // Distance to target calculated with field-relative coordinates
    double distance = pose.getTranslation().getDistance(TARGET_POSITION);

    // Time of flight (simple model, no air resistance, shot is flat, shooter velocity is constant)
    double timeOfFlight =
        ShootingConstants.getTimeOfFlight(
            ShootingUtil.getApproximateVirtualDistanceToHub(
                pose, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));

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

    double confidence = probability * 100; // Scale to percentage for easier interpretation
    return confidence;
  }
}
