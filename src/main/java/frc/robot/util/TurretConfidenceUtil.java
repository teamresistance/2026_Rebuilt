package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;

public class TurretConfidenceUtil {

  private static SwerveDriveIO drive; // Default to real drive, can be overridden for testing

  public static void instantiateDrive(SwerveDriveIO drive) {
    TurretConfidenceUtil.drive = drive;
  }

  // Tune these to the robot
  private static final double SHOOTER_VELOCITY =
      18.0; // in m/s, assuming no air resistance and constant shooter speed for simplicity
  private static final double ERROR_STD_DEV =
      1.0; // tune the standard deviation for the robot's accuracy
  private static final double SHOOTER_HEIGHT =
      0.7; // in meters, height of the shooter from the ground - placeholder
  private static Translation2d TARGET_POSITION =
      ShootingUtil.getShootingTarget(drive.getPose()).getTranslation();

  public static double confidence = 0.0;

  // Call this in Robot.java to update confidence
  public static double calculateConfidence() {
    ;

    // Get current robot pose and velocity
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeeds();

    // Update target position based on current robot pose
    TARGET_POSITION = ShootingUtil.getShootingTarget(drive.getPose()).getTranslation();

    // Distance to target calculated with field-relative coordinates
    double distance = pose.getTranslation().getDistance(TARGET_POSITION);

    // Time of flight (simple model, no air resistance, shot is flat, shooter velocity is constant)
    double timeOfFlight =
        Math.sqrt(distance * distance + SHOOTER_HEIGHT * SHOOTER_HEIGHT) / SHOOTER_VELOCITY;

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

    confidence = probability;
    return confidence;
  }

  public double getConfidence() {
    return confidence;
  }

  public boolean isSafeToShoot(double threshold) {
    return confidence >= threshold;
  }
}
