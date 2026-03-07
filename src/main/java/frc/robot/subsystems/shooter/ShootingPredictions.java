package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShootingConstants;
import frc.robot.util.ShootingUtil;
import frc.robot.util.ShootingUtil.BallisticSolution;
import org.littletonrobotics.junction.Logger;

public class ShootingPredictions {

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
   * Predicted straight-line distance (meters) to the hub after the reload delay. Used as an input
   * to ballistic calculations. Updated by {@link #predictDistanceAndAngleAfterReload}.
   */
  private double predictedDistanceToHubAfterReload = 0;

  /**
   * Predicted field-relative angle (radians) to the hub after the reload delay. This is the angle
   * in the robot's field coordinate frame and is used by the ballistic solver.
   */
  private double predictedFieldRelativeAngleToHubAfterReload = 0;

  /**
   * Time delay (seconds) from when a shot starts until the next ball is ready to be launched
   * (reload time). Used to predict where the robot/target will be at the time of the next shot.
   */
  private static final double reloadTime = ShootingConstants.RELOAD_TIME;

  // 20ms loop
  private static final double LOOP_DT = 0.02;

  private Transform2d acceleration = new Transform2d();

  // Maximum allowed change per second
  private static final double MAX_HOOD_RATE_DEG_PER_SEC = 180.0;
  private static final double MAX_TURRET_RATE_DEG_PER_SEC = 360.0;
  private static final double MAX_LAUNCH_VELOCITY_RATE_MPS = 15.0;

  // Derived per-cycle limits
  private static final double MAX_HOOD_DELTA = MAX_HOOD_RATE_DEG_PER_SEC * LOOP_DT;

  private static final double MAX_TURRET_DELTA = MAX_TURRET_RATE_DEG_PER_SEC * LOOP_DT;

  private static final double MAX_LAUNCH_VELOCITY_DELTA = MAX_LAUNCH_VELOCITY_RATE_MPS * LOOP_DT;

  private static double rateLimit(double current, double target, double maxDelta) {
    double delta = MathUtil.inputModulus(target - current, -180.0, 180.0);

    if (Math.abs(delta) > maxDelta) {
      delta = Math.copySign(maxDelta, delta);
    }

    return current + delta;
  }

  /**
   * Update all shooting-related parameters using the current robot motion and target geometry.
   *
   * <p>High-level contract: - Inputs: current distance to the hub, field-relative angle to the hub,
   * and the robot's chassis speeds (vx, vy). - Side effects: updates the static output fields
   * (angles, launchVelocity, desiredAngularAcceleration) and records them via the Logger. - Error
   * modes: none; this is a best-effort calculation and overwrites previous values.
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
    // Call the master method with zero acceleration
    updateShootingParameters(
        distanceToHub, fieldRelativeAngleToHub, chassisSpeeds, new Transform2d(), robotPose);
  }

  public void updateShootingParameters(
      double distanceToHub,
      double fieldRelativeAngleToHub,
      ChassisSpeeds chassisSpeeds,
      Transform2d acceleration,
      Pose2d robotPose) {

    // Convert the supplied robot-relative chassis speeds into field-relative
    // components before using them for prediction and in the ballistic solver.
    // This matches the expectations of FastBallisticCalculator which assumes
    // vx/vy are expressed in the field (floor) frame.
    Rotation2d robotRotation = robotPose.getRotation();
    double cos = Math.cos(robotRotation.getRadians());
    double sin = Math.sin(robotRotation.getRadians());

    // Robot-relative velocities (vx, vy) are rotated into the field frame.
    double vxField = chassisSpeeds.vxMetersPerSecond * cos - chassisSpeeds.vyMetersPerSecond * sin;
    double vyField = chassisSpeeds.vxMetersPerSecond * sin + chassisSpeeds.vyMetersPerSecond * cos;

    double axField = acceleration.getX() * cos - acceleration.getY() * sin;
    double ayField = acceleration.getX() * sin + acceleration.getY() * cos;

    // Compute ballistic solution for predicted range/angle using current robot
    // velocity in the field frame.
    BallisticSolution calculation =
        ShootingUtil.computeBallistics(
            distanceToHub,
            Math.toDegrees(fieldRelativeAngleToHub),
            vxField,
            vyField,
            axField,
            ayField,
            robotPose);

    // Store outputs from the calculators into this manager's public fields.
    double newVertical = calculation.hoodAngleDeg();
    double newOffset = calculation.deltaAzimuthDeg();
    double newTotal = newOffset + Math.toDegrees(predictedFieldRelativeAngleToHubAfterReload);
    double newLaunchVelocity = calculation.launchSpeed();

    // Apply rate limiting
    verticalShootingAngle = rateLimit(verticalShootingAngle, newVertical, MAX_HOOD_DELTA);

    horizontalTotalShootingAngle =
        rateLimit(horizontalTotalShootingAngle, newTotal, MAX_TURRET_DELTA);

    launchVelocity = rateLimit(launchVelocity, newLaunchVelocity, MAX_LAUNCH_VELOCITY_DELTA);

    // Keep offset consistent with total
    horizontalOffsetShootingAngle =
        horizontalTotalShootingAngle - Math.toDegrees(predictedFieldRelativeAngleToHubAfterReload);
    desiredAngularVelocity = ShootingUtil.computeMotorAdjustment(calculation.launchSpeed());

    // Record outputs for logging and tuning/debugging purposes.
    Logger.recordOutput("Shooting/VerticalShootingAngle", verticalShootingAngle);
    Logger.recordOutput(
        "Shooting/PredictedFieldRelativeAngleToHubAfterReload",
        Math.toDegrees(predictedFieldRelativeAngleToHubAfterReload));
    Logger.recordOutput("Shooting/HorizontalOffsetShootingAngle", horizontalOffsetShootingAngle);
    Logger.recordOutput("Shooting/HorizontalTotalShootingAngle", horizontalTotalShootingAngle);
    Logger.recordOutput("Shooting/LaunchVelocity", launchVelocity);
  }

  /**
   * @return most recently computed vertical shooting angle (degrees)
   */
  public double getVerticalShootingAngle() {
    return verticalShootingAngle;
  }

  /**
   * @return most recently computed horizontal offset shooting angle (degrees)
   */
  public double getHorizontalOffsetShootingAngle() {
    return horizontalOffsetShootingAngle;
  }

  /**
   * @return most recently computed launch velocity (m/s)
   */
  public double getLaunchVelocity() {
    return launchVelocity;
  }

  /**
   * @return most recently computed desired angular velocity for the shooter motor (rad/s)
   */
  public double getDesiredAngularVelocity() {
    return desiredAngularVelocity;
  }

  /**
   * @return the total horizontal shooting angle (degrees)
   */
  public double getHorizontalTotalShootingAngle() {
    return horizontalTotalShootingAngle;
  }
}
