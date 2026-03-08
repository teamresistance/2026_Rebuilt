package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            }else if (calcMode == ShootingStyle.CALC) {
        // 1. Get current robot state from Drive
        Pose2d robotPose = drive.getPose();
        ChassisSpeeds speeds = drive.getChassisSpeeds();

        // 2. Calculate Target Geometry
        var targetPose = ShootingUtil.getShootingTarget(robotPose);
        Translation2d hub = targetPose.getTranslation();
        Translation2d robot = robotPose.getTranslation();

        double dx = hub.getX() - robot.getX();
        double dy = hub.getY() - robot.getY();
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

    }

    shooter.setTurretTarget(turretAngleDeg);
    shooter.setHoodTarget(hoodAngleDeg);
  }
}
