package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class ShootingUtil {

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_HOOD_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toHoodRevs(double degrees) {
    return degrees * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toTurretDegrees(double motorRevs) {
    return motorRevs / Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  /**
   * Returns the motor revolutions converted to an equal angle. There are {@code
   * Constants.SHOOTER_TURRET_REVS_PER_DEG} motor shaft revolutions per one degree of pitch.
   */
  public static double toTurretRevs(double degrees) {
    return degrees * Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  /**
   * Returns the angle from the predicted shooting position to the goal center.
   *
   * @param robotPose current robot pose
   * @param robotVelocity robot velocity represented
   * @param estimatedAirtime estimated airtime of the projectile (seconds)
   * @return angle to point at in order to look at the goal
   */
  public static double getAngleToAim(
      Pose2d robotPose, Transform2d robotVelocity, double estimatedAirtime) {
    Pose2d shootingFrom = robotPose.plus(robotVelocity.times(estimatedAirtime));
    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    double dx = goalPose.getX() - shootingFrom.getX();
    double dy = goalPose.getY() - shootingFrom.getY();
    return Math.toDegrees(Math.atan2(dy, dx));
  }

  /**
   * Returns the angle from the predicted shooting position to the goal center, assuming airtime is
   * reasonably close to 1s
   *
   * @param robotPose current robot pose
   * @param robotVelocity robot velocity represented
   * @return angle to point at in order to look at the goal
   */
  public static double getAngleToAim(Pose2d robotPose, Transform2d robotVelocity) {
    Pose2d shootingFrom = robotPose.plus(robotVelocity);
    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    double dx = goalPose.getX() - shootingFrom.getX();
    double dy = goalPose.getY() - shootingFrom.getY();
    return Math.toDegrees(Math.atan2(dy, dx));
  }
}
