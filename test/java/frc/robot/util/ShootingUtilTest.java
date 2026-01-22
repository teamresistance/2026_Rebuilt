// java
package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("ShootingUtil Tests")
public class ShootingUtilTest {

  private static final double EPS = 1e-6;

  @Test
  @DisplayName("Stationary robot: angle to origin")
  void stationaryShouldPointToOrigin() {
    Pose2d robotPose = new Pose2d(1.0, 0.0, new Rotation2d());
    Transform2d robotVelocity = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());

    double actual = ShootingUtil.getAngleToAim(robotPose, robotVelocity);
    double expected = Math.toDegrees(Math.atan2(0.0 - robotPose.getY(), 0.0 - robotPose.getX()));

    assertEquals(expected, actual, EPS);
  }

  @Test
  @DisplayName("Moving towards goal: one second prediction")
  void movingTowardsGoal() {
    Pose2d robotPose = new Pose2d(2.0, 0.0, new Rotation2d());
    Transform2d robotVelocity =
        new Transform2d(new Translation2d(-1.0, 0.0), new Rotation2d()); // moves left 1m/s

    double actual = ShootingUtil.getAngleToAim(robotPose, robotVelocity);
    Pose2d shootingFrom = robotPose.plus(robotVelocity); // predicted after ~1s
    double expected =
        Math.toDegrees(Math.atan2(0.0 - shootingFrom.getY(), 0.0 - shootingFrom.getX()));

    assertEquals(expected, actual, EPS);
  }
}
