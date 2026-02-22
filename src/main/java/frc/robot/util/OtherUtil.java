package frc.robot.util;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class OtherUtil {

  public static Transform2d getClimberAlignPos(boolean left) {
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

    return Transform2d.kZero;
  }
}
