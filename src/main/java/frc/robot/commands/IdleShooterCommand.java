package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.util.FastBallisticCalculator;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    double launchSpeed = shooter.getLaunchVelocity();
    double turretAngleDeg = shooter.getHorizontalTotalShootingAngle();
double turretAngleRad = Math.toRadians(turretAngleDeg);
    double hoodAngleRad = Math.toRadians(shooter.getVerticalShootingAngle());
    Transform2d accel = drive.getAcceleration();

    // 1. PREDICT RELEASE POSE (The "Muzzle" position)
    double vxRobot = drive.getVelocity().getX();
    double vyRobot = drive.getVelocity().getY();
    double ax = accel.getX();
    double ay = accel.getY();
    double R = ShootingConstants.RELOAD_TIME;

    double xRelease = robotPose.getX() + (vxRobot * R + 0.5 * ax * R * R);
    double yRelease = robotPose.getY() + (vyRobot * R + 0.5 * ay * R * R);
    double vxRobotAtRelease = vxRobot + ax * R;
    double vyRobotAtRelease = vyRobot + ay * R;

    Pose2d releasePose = new Pose2d(xRelease, yRelease, robotPose.getRotation());
    Logger.recordOutput("Shooter/Visuals/ReleasePose", releasePose);

    // 2. CALCULATE INITIAL FIELD VELOCITY AT RELEASE
    double vFloorRelative = launchSpeed * Math.cos(hoodAngleRad);
    double vzInitial = launchSpeed * Math.sin(hoodAngleRad);

    double vxField = vFloorRelative * Math.cos(turretAngleRad) + vxRobotAtRelease;
    double vyField = vFloorRelative * Math.sin(turretAngleRad) + vyRobotAtRelease;

    // 3. USE THE SOLVER'S OWN RK2 FOR THE PREDICTION
    // This ensures the green dot on the map is exactly where the solver THINKS it's going.
    double[] landing =
        FastBallisticCalculator.predictLandingPose(xRelease, yRelease, vxField, vyField, vzInitial);

    Pose2d predictedLanding =
        new Pose2d(landing[0], landing[1], Rotation2d.fromRadians(Math.atan2(vyField, vxField)));

    // 4. LOGGING & MARKERS
    Logger.recordOutput("Shooter/Visuals/PredictedLanding", predictedLanding);

    // "Aim Line" from the future release point to the landing spot
    Logger.recordOutput(
        "Shooter/Visuals/ShotTrajectoryLine", new Pose2d[] {releasePose, predictedLanding});

    // Existing hardware targets
    shooter.setTurretTarget(turretAngleDeg);
    shooter.setHoodTarget(Math.toDegrees(hoodAngleRad));
  }
}
