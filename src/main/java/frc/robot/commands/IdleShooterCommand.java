package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final LoggedTunableNumber tunableHood =
      new LoggedTunableNumber("Shooter/TunableHood", Constants.SHOOTER_HOOD_MIN_PITCH);

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    double distance =
        ShootingUtil.getVirtualDistanceToTarget(drive.getPose(), drive.getChassisSpeeds());
    double turretAngle =
        ShootingUtil.getAngleToAim(
            drive.getPose(),
            drive.getChassisSpeeds(),
            ShootingConstants.getTimeOfFlight(distance) - 0.4);
    double hoodAngle = ShootingConstants.getHoodAngle(distance);

    shooter.setTurretTarget(turretAngle);
    if (!Constants.TUNING_MODE) {
      shooter.setHoodTarget(hoodAngle);
    } else {
      shooter.setHoodTarget(tunableHood.get());
    }

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
  }
}
