package frc.robot.util.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShootingConstants;
import frc.robot.Constants;
import frc.robot.FieldConstants;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * ShootingManager - Coordinates ballistic calculations and motor adjustment
 * computations
 * to produce the runtime shooting parameters used by shooter control.
 *
 * <p>
 * This class is a static utility that maintains the most recently computed
 * shooting
 * parameters (angles, launch velocity, and desired motor acceleration). It
 * performs the
 * following high-level steps when updating shooting parameters:
 * - Predict the robot-relative distance and angle to the hub after the reload
 * delay
 * - Compute ballistic trajectory parameters needed to hit the hub
 * - Compute motor adjustments required to restore launcher wheel energy after
 * firing
 *
 * <p>
 * The class stores results in public static fields and exposes simple getters
 * for
 * other subsystems or commands. All fields are updated by
 * {@link #updateShootingParameters}.
 */
public class ShootingManager {

  /**
   * Vertical (elevation) angle to set the shooter to, in degrees.
   * Updated by {@link #updateShootingParameters} after ballistic calculation.
   */
  private static double verticalShootingAngle = 0;

  /**
   * Horizontal offset angle (floor-plane) to aim the shooter, in degrees.
   * Accounts for
   * lateral offsets between robot heading and target after predicting motion
   * during reload.
   */
  private static double horizontalOffsetShootingAngle = 0;

  /**
   * Total horizontal shooting angle (floor-plane) in degrees, combining the
   * robot's
   * current heading and the required offset to aim at the target. This is the
   * absolute
   * angle the shooter should be set to in the floor frame.
   */
  private static double horizontalTotalShootingAngle = 0;

  /**
   * Computed launch velocity (m/s) required to reach the target using the current
   * predicted range and offsets. Obtained from {@code FastBallisticCalculator}.
   */
  private static double launchVelocity = 0;

  /**
   * Desired angular acceleration (rad/s^2) the shooter motor should apply to
   * compensate
   * for energy lost during a shot. Computed by {@code MotorAdjustmentCalculator}.
   */
  private static double desiredAngularAcceleration = 0;

  /**
   * Desired angular velocity of the shooter wheel after adjustment (rad/s) -
   * target speed post-launch. Computed by {@code MotorAdjustmentCalculator}.
   */
  private static double desiredAngularVelocity = 0;

  /**
   * Predicted straight-line distance (meters) to the hub after the reload delay.
   * Used as
   * an input to ballistic calculations. Updated by
   * {@link #predictDistanceAndAngleAfterReload}.
   */
  private static double predictedDistanceToHubAfterReload = 0;

  /**
   * Predicted field-relative angle (radians) to the hub after the reload delay.
   * This is the
   * angle in the robot's field coordinate frame and is used by the ballistic
   * solver.
   */
  private static double predictedFieldRelativeAngleToHubAfterReload = 0;

  /**
   * Time delay (seconds) from when a shot starts until the next ball is ready to
   * be
   * launched (reload time). Used to predict where the robot/target will be at the
   * time of the next shot.
   */
  private static final double reloadTime = ShootingConstants.RELOAD_TIME;

  // Private constructor: utility class only.
  private ShootingManager() {
  }

  /**
   * Update all shooting-related parameters using the current robot motion and
   * target
   * geometry.
   *
   * <p>
   * High-level contract:
   * - Inputs: current distance to the hub, field-relative angle to the hub, and
   * the robot's
   * chassis speeds (vx, vy).
   * - Side effects: updates the static output fields (angles, launchVelocity,
   * desiredAngularAcceleration) and records them via the Logger.
   * - Error modes: none; this is a best-effort calculation and overwrites
   * previous values.
   *
   * @param distanceToHub           current straight-line distance (meters) to the
   *                                hub
   * @param fieldRelativeAngleToHub current field-relative angle (radians) to the
   *                                hub
   * @param chassisSpeeds           current robot chassis speeds (m/s)
   */
  public static void updateShootingParameters(double distanceToHub, double fieldRelativeAngleToHub,
      ChassisSpeeds chassisSpeeds) {

    // Predict where the robot/target will be after the reload delay so ballistic
    // calculations account for robot motion during the reload.
    predictDistanceAndAngleAfterReload(chassisSpeeds, distanceToHub, fieldRelativeAngleToHub);

    // Compute ballistic solution for predicted range/angle using current robot
    // velocity.
    FastBallisticCalculator.computeBallistics(
        predictedDistanceToHubAfterReload,
        predictedFieldRelativeAngleToHubAfterReload,
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond);

    // Compute the motor adjustment required to restore wheel energy after firing.
    MotorAdjustmentCalculator.computeMotorAdjustment();

    // Store outputs from the calculators into this manager's public fields.
    verticalShootingAngle = FastBallisticCalculator.thetaDeg;
    horizontalOffsetShootingAngle = FastBallisticCalculator.deltaFloorAngleDeg;
    horizontalTotalShootingAngle = horizontalOffsetShootingAngle
        + Math.toDegrees(predictedFieldRelativeAngleToHubAfterReload);
    launchVelocity = FastBallisticCalculator.vTotalNew;
    desiredAngularAcceleration = MotorAdjustmentCalculator.desiredAngularAcceleration;
    desiredAngularVelocity = MotorAdjustmentCalculator.desiredAngularVelocity;

    // Record outputs for logging and tuning/debugging purposes.
    Logger.recordOutput("Shooting/VerticalShootingAngle", verticalShootingAngle);
    Logger.recordOutput("Shooting/HorizontalOffsetShootingAngle", horizontalOffsetShootingAngle);
    Logger.recordOutput("Shooting/LaunchVelocity", launchVelocity);
    Logger.recordOutput("Shooting/DesiredAngularAcceleration", desiredAngularAcceleration);
  }

  /**
   * @return most recently computed vertical shooting angle (degrees)
   */
  public static double getVerticalShootingAngle() {
    return verticalShootingAngle;
  }

  /**
   * @return most recently computed horizontal offset shooting angle (degrees)
   */
  public static double getHorizontalOffsetShootingAngle() {
    return horizontalOffsetShootingAngle;
  }

  /**
   * @return most recently computed launch velocity (m/s)
   */
  public static double getLaunchVelocity() {
    return launchVelocity;
  }

  /**
   * @return most recently computed desired angular acceleration for the shooter
   *         motor
   *         (rad/s^2)
   */
  public static double getDesiredAngularAcceleration() {
    return desiredAngularAcceleration;
  }

  /**
  * @return most recently computed desired angular velocity for the shooter motor
  *         (rad/s)
  */
  public static double getDesiredAngularVelocity() {
    return desiredAngularVelocity;
  }

  /**
   * Predicts the robot-to-hub distance and field-relative angle at the time when
   * the next
   * shot will be available (after {@link #reloadTime}). The prediction uses the
   * linear
   * components of the robot's chassis speed to estimate displacement during the
   * reload.
   *
   * <p>
   * Outputs are written to {@code predictedDistanceToHubAfterReload} and
   * {@code predictedFieldRelativeAngleToHubAfterReload} and also recorded via
   * Logger.
   *
   * @param chassisSpeeds        current chassis speeds (m/s)
   * @param currentDistanceToHub current straight-line distance to the hub
   *                             (meters)
   * @param currentAngleToHub    current field-relative angle to the hub (radians)
   */
  public static void predictDistanceAndAngleAfterReload(ChassisSpeeds chassisSpeeds, double currentDistanceToHub,
      double currentAngleToHub) {
    double predictedX = chassisSpeeds.vxMetersPerSecond * reloadTime;
    double predictedY = chassisSpeeds.vyMetersPerSecond * reloadTime;

    // Hypotenuse gives the predicted straight-line distance after applying the
    // translation due to robot motion during reload.
    predictedDistanceToHubAfterReload = Math.hypot(currentDistanceToHub + predictedX, predictedY);

    // Angle relative to the robot/field after the predicted displacement.
    predictedFieldRelativeAngleToHubAfterReload = Math.atan2(predictedY, currentDistanceToHub + predictedX);

    Logger.recordOutput("Shooting/PredictedDistanceToHubAfterReload", predictedDistanceToHubAfterReload);
    Logger.recordOutput("Shooting/PredictedFieldRelativeAngleToHubAfterReload",
        Math.toDegrees(predictedFieldRelativeAngleToHubAfterReload));
  }

  /**
   * @return the predicted distance to the hub after reload (meters)
   */
  public static double getPredictedDistanceToHubAfterReload() {
    return predictedDistanceToHubAfterReload;
  }

  /**
   * @return the predicted field-relative angle to the hub after reload (radians)
   */
  public static double getPredictedFieldRelativeAngleToHubAfterReload() {
    return predictedFieldRelativeAngleToHubAfterReload;
  }

  /**
   * @return the total horizontal shooting angle (degrees)
   */
  public static double getHorizontalTotalShootingAngle() {
    return horizontalTotalShootingAngle;
  }

  /**
   * Utility methods for converting between motor revolutions and shooter angles,
   * based on the known gear ratios of the shooter mechanism.
   * These methods allow commands and subsystems to work in intuitive angle units
   * while the ShooterIO implementation can work in motor revolutions, abstracting
   * away the conversion details.
   * The conversion factors are defined in {@code Constants} and are based on the
   * physical design of the shooter mechanisms,
   * (e.g., how many motor revolutions correspond to one degree of hood pitch or
   * turret yaw).
   */
  public static class Conversions {
    /**
     * Returns the motor revolutions converted to an equal angle. There are {@code
     * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree
     * of pitch.
     */
    public static double toHoodDegrees(double motorRevs) {
      return motorRevs / Constants.SHOOTER_HOOD_REVS_PER_DEG;
    }

    /**
     * Returns the angle converted to an equal amount of motor revolutions. There
     * are {@code
     * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree
     * of pitch.
     */
    public static double toHoodRevs(double degrees) {
      return degrees * Constants.SHOOTER_HOOD_REVS_PER_DEG;
    }

    /**
     * Returns the motor revolutions converted to an equal angle. There are {@code
     * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree
     * of yaw.
     */
    public static double toTurretDegrees(double motorRevs) {
      return motorRevs / Constants.SHOOTER_TURRET_REVS_PER_DEG;
    }

    /**
     * Returns the angle converted to an equal amount of motor revolutions. There
     * are {@code
     * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree
     * of yaw.
     */
    public static double toTurretRevs(double degrees) {
      return degrees * Constants.SHOOTER_TURRET_REVS_PER_DEG;
    }
  }

  public static class SimulationAndState {

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

      boolean isTop = pose.getY() >= FieldConstants.TOP_BOTTOM_SPLIT_Y;

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
  }

}