package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class BumpUtil {

  /**
   * Returns if transform(s) of the robot are in the bump auto-rotate zone, as defined with the BUMPZONE_START
   * and BUMPZONE_END values.
   */
  public static boolean inBumpZone(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    ChassisSpeeds speeds = speedsSupplier.get();
    Pose2d basePose = poseSupplier.get();

    // check multiple scaled virtual poses ahead of the robot (0.25x, 0.5x, 0.75x)
    double[] scales = {0.25, 0.5, 0.75};
    boolean isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    for (double s : scales) {
      Pose2d pose =
          basePose.plus(
              new Transform2d(
                  speeds.vxMetersPerSecond * s, speeds.vyMetersPerSecond * s, Rotation2d.kZero));
      Logger.recordOutput(String.format("Bump/Bump Zone Virtual Pose (scale=%.2f)", s), pose);

      if (isBlue) {
        if (pose.getX() < FieldConstants.BUMPZONE_END_BLUE
            && pose.getX() > FieldConstants.BUMPZONE_START_BLUE) {
          return true;
        }
      } else {
        if (pose.getX() < FieldConstants.BUMPZONE_END_RED
            && pose.getX() > FieldConstants.BUMPZONE_START_RED) {
          return true;
        }
      }
    }

    return false;
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
