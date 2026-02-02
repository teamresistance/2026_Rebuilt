package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class ShootCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public ShootCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
  }

  @Override
  public void execute() {

    double distance =
        ShootingUtil.getVirtualDistanceToHub(drive.getPose(), drive.getChassisSpeeds());
    double turretAngle =
        ShootingUtil.getAngleToAim(
            drive.getPose(), drive.getChassisSpeeds(), ShootingConstants.getTimeOfFlight(distance));
    double hoodAngle = ShootingConstants.getHoodAngle(distance);

    shooter.setTurretTarget(turretAngle);
    shooter.setHoodTarget(hoodAngle);
    shooter.runFlywheelAtRPS(ShootingConstants.getRPS(distance));

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);

    // TODO: Control the feeder based on the RPS being good/bad and the turret and hood being ready
  }
}
