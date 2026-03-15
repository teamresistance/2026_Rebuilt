package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShootingStyle;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingMaps;
import frc.robot.subsystems.shooter.ShootingPredictions;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;
  private final ShootingStyle calcMode;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter, ShootingStyle _calcMode) {
    drive = _drive;
    shooter = _shooter;
    calcMode = _calcMode;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double turretAngleDeg = 0;
    double hoodAngleDeg = 0;

    if (calcMode == ShootingStyle.MAPS) {
      double distance =
          ShootingUtil.getVirtualDistanceToTarget(drive.getPose(), drive.getChassisSpeeds());
      turretAngleDeg =
          ShootingUtil.getAngleToAim(
              drive.getPose(), drive.getChassisSpeeds(), ShootingMaps.getTimeOfFlight(distance));
      hoodAngleDeg = ShootingMaps.getHoodAngle(distance);

      Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
    } else if (calcMode == ShootingStyle.CALC) {
      // 1. Get current robot state from Drive
      Pose2d robotPose = drive.getPose();
      ChassisSpeeds speeds = drive.getChassisSpeeds();

      // 2. Calculate Target Geometry
      // Offset robot center to turret center in field frame
      Translation2d turretOffset =
          Constants.ROBOT_TO_TURRET
              .plus(new Transform2d(0, 0, robotPose.getRotation()))
              .getTranslation();
      Translation2d turretPos = robotPose.getTranslation().plus(turretOffset);

      var targetPose = ShootingUtil.getShootingTarget(robotPose);
      Translation2d hub = targetPose.getTranslation();

      double dx = hub.getX() - turretPos.getX();
      double dy = hub.getY() - turretPos.getY();
      double distanceToHub = Math.hypot(dx, dy);
      double fieldRelativeAngleToHub = Math.atan2(dy, dx);

      // 3. Update the global calculator (Stateful calculation)
      // We pass the acceleration stored in the shooter's IO
      ShootingPredictions.getCalculator()
          .updateShootingParameters(
              distanceToHub, fieldRelativeAngleToHub, speeds, drive.getAcceleration(), robotPose);

      // 4. Extract results and set targets
      // Turret needs to be robot-relative for the hardware/PID
      turretAngleDeg =
          ShootingPredictions.getCalculator().getHorizontalTotalShootingAngle()
              - robotPose.getRotation().getDegrees();
      hoodAngleDeg = ShootingPredictions.getCalculator().getVerticalShootingAngle();
      // 5. Visualization — turret pose and predicted ball trajectory
      Translation2d turretVizOffset =
          (Constants.ROBOT_TO_TURRET.plus(new Transform2d(0, 0, robotPose.getRotation())))
              .getTranslation();
      Translation2d turretVizPos = robotPose.getTranslation().plus(turretVizOffset);
      // 5. Visualization — predicted release pose and ball trajectory
      Rotation2d robotRotation = robotPose.getRotation();
      double cosR = Math.cos(robotRotation.getRadians());
      double sinR = Math.sin(robotRotation.getRadians());
      double vxField = speeds.vxMetersPerSecond * cosR - speeds.vyMetersPerSecond * sinR;
      double vyField = speeds.vxMetersPerSecond * sinR + speeds.vyMetersPerSecond * cosR;

      Transform2d accel = drive.getAcceleration();
      double axField = accel.getX() * cosR - accel.getY() * sinR;
      double ayField = accel.getX() * sinR + accel.getY() * cosR;

      double reloadTime = Constants.ShootingConstants.RELOAD_TIME;
      double turretRelX =
          turretVizPos.getX() + vxField * reloadTime + 0.5 * axField * reloadTime * reloadTime;
      double turretRelY =
          turretVizPos.getY() + vyField * reloadTime + 0.5 * ayField * reloadTime * reloadTime;

      Pose2d turretPose = new Pose2d(turretVizPos, robotPose.getRotation());
      Pose2d releasePose = new Pose2d(turretRelX, turretRelY, robotPose.getRotation());

      // Robot velocity at release (field frame)
      double vxRobotRelease = vxField + axField * reloadTime;
      double vyRobotRelease = vyField + ayField * reloadTime;

      // Use raw solver output for trajectory visualization
      ShootingUtil.BallisticSolution raw = ShootingPredictions.getCalculator().getLastRawSolution();

      double hoodRad = Math.toRadians(raw.hoodAngleDeg());
      double turretFieldRad = Math.toRadians(raw.deltaAzimuthDeg()) + fieldRelativeAngleToHub;
      double launchSpeed = raw.launchSpeed();

      // Shooter-frame floor velocity components
      double vFloorShooter = launchSpeed * Math.cos(hoodRad);
      double vxShooter = vFloorShooter * Math.cos(turretFieldRad);
      double vyShooter = vFloorShooter * Math.sin(turretFieldRad);

      // Ground-frame launch velocity = shooter frame + robot velocity at release
      double vxLaunch = vxShooter + vxRobotRelease;
      double vyLaunch = vyShooter + vyRobotRelease;
      double vzLaunch = launchSpeed * Math.sin(hoodRad);

      double[] landing =
          ShootingUtil.predictLandingPose(turretRelX, turretRelY, vxLaunch, vyLaunch, vzLaunch);

      Logger.recordOutput("Shooter/TurretPose", turretPose);
      Logger.recordOutput("Shooter/ReleasePose", releasePose);
      Logger.recordOutput(
          "Shooter/PredictedLandingPose", new Pose2d(landing[0], landing[1], Rotation2d.kZero));
      Logger.recordOutput(
          "Shooter/TrajectoryLine",
          new Pose2d[] {releasePose, new Pose2d(landing[0], landing[1], Rotation2d.kZero)});
      Logger.recordOutput("Shooter/DistanceToHub", distanceToHub);
      Logger.recordOutput("Shooter/TurretPos", turretVizPos.toString());
      Logger.recordOutput(
          "Shooter/RotationsPerSecond",
          ShootingPredictions.getCalculator().getDesiredAngularVelocity());
    }

    shooter.setTurretTarget(turretAngleDeg);
    shooter.setHoodTarget(hoodAngleDeg);
  }
}
