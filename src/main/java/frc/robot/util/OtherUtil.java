package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class OtherUtil {

  public static Pose2d getClimberAlignPos(boolean left) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return left
            ? FieldConstants.BLUE_LEFT_CLIMBER_ALIGN
            : FieldConstants.BLUE_RIGHT_CLIMBER_ALIGN;
      } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return left
            ? FieldConstants.RED_LEFT_CLIMBER_ALIGN
            : FieldConstants.RED_RIGHT_CLIMBER_ALIGN;
      }
    }

    return Pose2d.kZero;
  }

  public static Pose2d getClimberAlignPos(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return (pose.getY() > FieldConstants.BLUE_GOAL_CENTER.getY())
            ? FieldConstants.BLUE_LEFT_CLIMBER_ALIGN
            : FieldConstants.BLUE_RIGHT_CLIMBER_ALIGN;
      } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return (pose.getY() < FieldConstants.RED_GOAL_CENTER.getY())
            ? FieldConstants.RED_LEFT_CLIMBER_ALIGN
            : FieldConstants.RED_RIGHT_CLIMBER_ALIGN;
      }
    }

    return Pose2d.kZero;
  }
}
