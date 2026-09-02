package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShiftUtil;
import frc.robot.util.ShootingUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final LoggedTunableNumber tunableHood =
      new LoggedTunableNumber("Shooter/TunableHood", Constants.SHOOTER_HOOD_MIN_PITCH);

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;
  private final BooleanSupplier fwdSup;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter, BooleanSupplier _fwd) {
    drive = _drive;
    shooter = _shooter;
    fwdSup = _fwd;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    double distance =
        ShootingUtil.getVirtualDistanceToTarget(
            drive.getPose(),
            drive
                .getChassisSpeeds()
                .plus(drive.getAcceleration().times(Constants.ACCELERATION_SOTM_SCALAR)),
          false);
    double turretAngle =
        ShootingUtil.getAngleToAim(
            drive.getPose(),
            drive.getChassisSpeeds(),
            ShootingConstants.getTimeOfFlight(distance),
          false);
    double hoodAngle = ShootingConstants.getHoodAngle(distance);

    if (distance < Constants.MIN_DISTANCE_METERS) {
      Constants.tooClose = true;
    } else {
      Constants.tooClose = false;
    }

    //     will only aim turret when needed, so when (almost) active or when commanding to shoot
    if (ShiftUtil.withinTwoSecondsOfNextShift()
        || ShiftUtil.isOurs(ShiftUtil.getShift())
        || shooter.isShooting()) {
      shooter.setTurretTarget(turretAngle, drive.getChassisSpeeds().omegaRadiansPerSecond);
      if (!Constants.TUNING_MODE) {
        shooter.setHoodTarget(hoodAngle);
      } else {
        shooter.setHoodTarget(tunableHood.get());
      }
    }

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
  }
}
