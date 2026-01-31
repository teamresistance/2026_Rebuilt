package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
   * Returns the angle from the predicted shooting position to the goal center.
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot velocity, field-relative
   * @param estimatedAirtime estimated airtime of the projectile (seconds)
   * @return an array in the format of [shot angle, distance to hub]
   */
  public static double[] getAngleToAim(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, double estimatedAirtime) {

    // Determine goal pose based on alliance
    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    double robotVelX = robotSpeeds.vxMetersPerSecond;
    double robotVelY = robotSpeeds.vyMetersPerSecond;
    double robotAngularVelocity = robotSpeeds.omegaRadiansPerSecond;

    Pose2d turretPose = robotPose.plus(Constants.ROBOT_TO_TURRET);
    Translation2d offsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());

    double vx_rot = -offsetField.getY() * robotAngularVelocity;
    double vy_rot = offsetField.getX() * robotAngularVelocity;
    double vx_total = robotVelX + vx_rot;
    double vy_total = robotVelY + vy_rot;

    Translation2d turretFuturePos =
        turretPose
            .getTranslation()
            .plus(new Translation2d(vx_total * estimatedAirtime, vy_total * estimatedAirtime));
    Translation2d toTarget = goalPose.getTranslation().minus(turretFuturePos);

    double turretAngleField = Math.atan2(toTarget.getY(), toTarget.getX());
    double turretAngleRobotRelative = turretAngleField - robotPose.getRotation().getRadians();
    turretAngleRobotRelative = MathUtil.angleModulus(turretAngleRobotRelative);

    return new double[] {
      Math.toDegrees(turretAngleRobotRelative),
      turretFuturePos.getDistance(goalPose.getTranslation())
    };
  }
}
