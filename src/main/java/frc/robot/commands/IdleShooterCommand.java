package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
      Pose2d robotPose = drive.getPose();
      ChassisSpeeds speeds = drive.getChassisSpeeds();

      // Compute target geometry, run ballistic solver, update rate-limited outputs
      ShootingPredictions.getCalculator()
          .updateDistanceAndAngle(robotPose, speeds, drive.getAcceleration());

      // Update AdvantageScope trajectory visualization
      ShootingPredictions.getCalculator()
          .updateAdvantageScope(robotPose, speeds, drive.getAcceleration());

      // Turret angle must be robot-relative for the hardware/PID
      turretAngleDeg =
          ShootingPredictions.getCalculator().getHorizontalTotalShootingAngle()
              - robotPose.getRotation().getDegrees();
      hoodAngleDeg = ShootingPredictions.getCalculator().getVerticalShootingAngle();

      Logger.recordOutput(
          "Shooter/Virtual Distance to Hub",
          ShootingPredictions.getCalculator().getDistanceToHub());
    }

    shooter.setTurretTarget(turretAngleDeg);
    shooter.setHoodTarget(90.0 - hoodAngleDeg);
  }
}
