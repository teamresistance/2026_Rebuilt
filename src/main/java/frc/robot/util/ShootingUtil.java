package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class ShootingUtil {

  private ShootingUtil() {}

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
   * @param robotSpeeds robot velocity
   * @param estimatedAirtime estimated airtime of the projectile (seconds)
   * @return the angle to aim to the turret at, in degrees
   */
  public static double getAngleToAim(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, double estimatedAirtime) {

    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d futureTurretPos =
        robotPose
            .getTranslation()
            .plus(turretOffsetField)
            .plus(fieldVelocity.times(estimatedAirtime));

    Translation2d toTarget = goalPose.getTranslation().minus(futureTurretPos);

    double turretAngleField = Math.atan2(toTarget.getY(), toTarget.getX());
    double turretAngleRobotRelative =
        MathUtil.angleModulus(turretAngleField - robotPose.getRotation().getRadians());

    Logger.recordOutput(
        "Shooter/Calculated Virtual Pose",
        new Pose2d(futureTurretPos.getX(), futureTurretPos.getY(), Rotation2d.kZero));

    return Math.toDegrees(turretAngleRobotRelative);
  }

  /**
   * Gets the distance from the "virtual turret pose" to the hub.
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot chassis speeds, field-relative
   * @return the distance between the hub and the virtual turret pose
   */
  public static double getVirtualDistanceToHub(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    double estimatedAirtime =
        ShootingConstants.getTimeOfFlight(
            ShootingUtil.getApproximateVirtualDistanceToHub(
                robotPose,
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)));

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d futureTurretPos =
        robotPose
            .getTranslation()
            .plus(turretOffsetField)
            .plus(fieldVelocity.times(estimatedAirtime));

    return futureTurretPos.getDistance(goalPose.getTranslation());
  }

  /**
   * Gets the distance to hub based on an average time-of-flight, ignoring rotation. This makes it
   * possible to take a "recursive" approach to calculating a perfect shot. Calculating a perfect
   * moving shot requires distance, which requires time of flight, which requires distance. This
   * skips the step of requiring an accurate time-of-flight and uses {@code
   * Constants.SHOOTING_APPROXIMATE_TOF} to get a mostly-there number, which is good enough - any
   * minor inaccuracy can be disregarded as the goal is very wide.
   */
  public static double getApproximateVirtualDistanceToHub(
      Pose2d robotPose, Translation2d velocity) {
    Pose2d goalPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      goalPose =
          switch (DriverStation.getAlliance().get()) {
            case Blue -> FieldConstants.BLUE_GOAL_CENTER;
            case Red -> FieldConstants.RED_GOAL_CENTER;
          };
    }

    return goalPose
        .getTranslation()
        .getDistance(
            robotPose.getTranslation().plus(velocity.times(Constants.SHOOTING_APPROXIMATE_TOF)));
  }
}
