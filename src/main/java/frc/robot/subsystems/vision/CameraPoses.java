package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraPoses {
  public static final Pose3d[] poses =
      new Pose3d[] {
        // Front Left
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(12.13),
                Units.inchesToMeters(12.34),
                Units.inchesToMeters(10.83)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(35.5))),
        // TODO: rest of me
        // Front Right
        new Pose3d(
            new Translation3d(0.268, -0.213, 0.267),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-12.63),
                Units.degreesToRadians(45.0))), // in radians btw
        // Back Right
        new Pose3d(
            new Translation3d(-0.235, -0.235, 0.267),
            new Rotation3d(
                0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(-45.0 - 90.0))),
        // Back Left
        new Pose3d(
            new Translation3d(-0.235, 0.235, 0.267),
            new Rotation3d(
                0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(45.0 + 90.0))),
      };
}
