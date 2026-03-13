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
      Pose2d turretPose = new Pose2d(turretVizPos, robotPose.getRotation());

      // Decompose launch velocity into field-frame components for trajectory prediction
      double launchSpeed = ShootingPredictions.getCalculator().getLaunchVelocity();
      double hoodRad =
          Math.toRadians(ShootingPredictions.getCalculator().getVerticalShootingAngle());
      double turretFieldRad =
          Math.toRadians(ShootingPredictions.getCalculator().getHorizontalTotalShootingAngle());

      double vxLaunch = launchSpeed * Math.cos(hoodRad) * Math.cos(turretFieldRad);
      double vyLaunch = launchSpeed * Math.cos(hoodRad) * Math.sin(turretFieldRad);
      double vzLaunch = launchSpeed * Math.sin(hoodRad);

      double[] landing =
          ShootingUtil.predictLandingPose(
              turretVizPos.getX(), turretVizPos.getY(), vxLaunch, vyLaunch, vzLaunch);

      // Log turret pose and predicted landing point
      Logger.recordOutput("Shooter/TurretPose", turretPose);
      Logger.recordOutput(
          "Shooter/PredictedLandingPose", new Pose2d(landing[0], landing[1], Rotation2d.kZero));

      // Log trajectory line as a Pose2d array (start = turret, end = landing)
      // AdvantageScope renders this as a line when logged as a pose array
      Logger.recordOutput(
          "Shooter/TrajectoryLine",
          new Pose2d[] {turretPose, new Pose2d(landing[0], landing[1], Rotation2d.kZero)});

      Logger.recordOutput("Shooter/DistanceToHub", distanceToHub);
      Logger.recordOutput("Shooter/TurretPos", turretVizPos.toString());
    }

    shooter.setTurretTarget(turretAngleDeg);
    shooter.setHoodTarget(hoodAngleDeg);
  }
}
