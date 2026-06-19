package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.function.DoubleSupplier;

public class ContinuousLimelightPoseCommand extends Command {

  // ccw positive here, turret is ccw negative, oops
  private final DoubleSupplier turretAngleSupplier;

  private final VisionIOLimelight limelight;

  public ContinuousLimelightPoseCommand(VisionIOLimelight ll, DoubleSupplier turretAngleSupplier) {
    this.limelight = ll;
    this.turretAngleSupplier = turretAngleSupplier;
    addRequirements(ll);
  }

  @Override
  public void execute() {

    // center is at turret x, turret y, and limelight z
    Pose3d centerOfTurret =
        new Pose3d(
            new Translation3d(
                Constants.ROBOT_TO_TURRET.getX(),
                Constants.ROBOT_TO_TURRET.getY(),
                Constants.LIMELIGHT_HEIGHT),
            new Rotation3d() // nothing yet
            );

    // to apply the turret's current ROTATION ONLY. turret angle is negative because turret +/- was
    // done wrong, and 90 deg is applied to counteract that turret zero = facing right
    Transform3d turretYaw =
        new Transform3d(
            new Translation3d(),
            new Rotation3d(
                0, 0, Units.degreesToRadians(-turretAngleSupplier.getAsDouble() + 90.0)));

    // turret center with rotation
    Pose3d rotatedTurretFrame = centerOfTurret.transformBy(turretYaw);

    // offset from turret center, the limelight transform is applied at an angle
    limelight.updateCameraTransform(
        "limelight", rotatedTurretFrame.transformBy(Constants.TURRET_TO_LIMELIGHT));
  }
}
