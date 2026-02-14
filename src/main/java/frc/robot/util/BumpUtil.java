package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

public class BumpUtil {

  /**
   * Returns if the robot is inside the bump auto-rotate zone, as defined with the BUMPZONE_START
   * and BUMPZONE_END values.
   */
  public static boolean inBumpZone(Supplier<Pose2d> poseSupplier) {
    Pose2d pose = poseSupplier.get();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return pose.getX() < FieldConstants.BUMPZONE_END_BLUE
            && pose.getX() > FieldConstants.BUMPZONE_START_BLUE; // if blue
      }
    }
    return pose.getX() < FieldConstants.BUMPZONE_END_RED
        && pose.getX() > FieldConstants.BUMPZONE_START_RED; // if not blue, its red...
  }

  /**
   * Gets the closest diagonal angle from the robot's current heading for use in crossing the bump
   * diagonally.
   */
  public static Rotation2d rotationToSnap(Supplier<Rotation2d> rotationSupplier) {
    Rotation2d rotation = rotationSupplier.get();
    double degrees = rotation.getDegrees();

    // only 45deg multiples
    double snappedDegrees = Math.round((degrees - 45.0) / 90.0) * 90.0 + 45.0;
    return Rotation2d.fromDegrees(snappedDegrees);
  }
}
