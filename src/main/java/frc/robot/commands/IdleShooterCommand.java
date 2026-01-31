package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.ShootingUtil;

public class IdleShooterCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
  }

  @Override
  public void execute() {
    double distance =
        ShootingUtil.getVirtualDistanceToHub(
            drive.getPose(), drive.getChassisSpeedsFieldRelative());
    shooter.setTurretTarget(
        ShootingUtil.getAngleToAim(
            drive.getPose(),
            drive.getChassisSpeeds(),
            ShootingConstants.getTimeOfFlight(distance)));
    shooter.setHoodTarget(ShootingConstants.getHoodAngle(distance));
  }
}
