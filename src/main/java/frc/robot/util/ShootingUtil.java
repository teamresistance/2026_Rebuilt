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
import java.util.function.Supplier;
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
   * gets the shooting target based on the robot pose, bounds for zones defined in FieldConstants
   */
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

  /**
   * Returns the angle from the predicted shooting position to the goal center. Goal is determined
   * by {@code FieldConstants.getShootingTarget()}
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot velocity
   * @param estimatedAirtime estimated airtime of the projectile (seconds)
   * @return the angle to aim to the turret at, in degrees
   */
  public static double getAngleToAim(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, double estimatedAirtime) {

    Pose2d goalPose = ShootingUtil.getShootingTarget(robotPose);
    Logger.recordOutput("Shooter/GoalPose", goalPose);

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

    Translation2d futureTurretPos = turretPos.plus(fieldVelocity.times(estimatedAirtime));

    Translation2d toTarget = goalPose.getTranslation().minus(futureTurretPos);
    double turretAngleField = Math.atan2(toTarget.getY(), toTarget.getX());
    double turretAngleRobotRelative =
        MathUtil.angleModulus(turretAngleField - robotPose.getRotation().getRadians());

    Logger.recordOutput(
        "Shooter/Calculated Virtual Pose",
        new Pose2d(futureTurretPos.getX(), futureTurretPos.getY(), Rotation2d.kZero));

    // code-front of the robot has turret 90deg right
    return Math.toDegrees(MathUtil.angleModulus(turretAngleRobotRelative + (Math.PI / 2)));
  }

  /**
   * Gets the distance from the "virtual turret pose" to the goal. Goal is determined by {@code
   * FieldConstants.getShootingTarget()}
   *
   * @param robotPose current robot pose
   * @param robotSpeeds robot chassis speeds, field-relative
   * @return the distance between the hub and the virtual turret pose
   */
  public static double getVirtualDistanceToTarget(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

    Pose2d goalPose = ShootingUtil.getShootingTarget(robotPose);
    Logger.recordOutput("Shooter/GoalPose", goalPose);

    Translation2d fieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d turretOffsetField =
        Constants.ROBOT_TO_TURRET.getTranslation().rotateBy(robotPose.getRotation());
    Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

    Translation2d futureTurretPos;
    double lookaheadDistance = turretPos.getDistance(goalPose.getTranslation());

    for (int i = 0; i < 20; i++) {
      double timeOfFlight = ShootingConstants.getTimeOfFlight(lookaheadDistance);
      futureTurretPos = turretPos.plus(fieldVelocity.times(timeOfFlight));
      lookaheadDistance = futureTurretPos.getDistance(goalPose.getTranslation());
    }

    return lookaheadDistance;
  }
}
