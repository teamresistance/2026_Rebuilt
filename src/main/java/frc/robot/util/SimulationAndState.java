package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

public class SimulationAndState {

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

    // TODO: Uncomment if the variable is needed, otherwise delete
    // boolean isTop = pose.getY() >= FieldConstants.TOP_BOTTOM_SPLIT_Y;

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
