package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShootingUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootCommand extends Command {

  private final LoggedTunableNumber tunableRPS = new LoggedTunableNumber("Shooter/TunableRPS", 50);

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;
  private final BooleanSupplier passingSup;

  public ShootCommand(SwerveDriveIO _drive, ShooterIO _shooter, BooleanSupplier _passing) {
    drive = _drive;
    shooter = _shooter;
    passingSup = _passing;
  }

  @Override
  public void execute() {

    double distance =
        ShootingUtil.getVirtualDistanceToTarget(
            drive.getPose(), drive.getChassisSpeeds(), passingSup.getAsBoolean());

    if (!Constants.TUNING_MODE) {
      shooter.runFlywheelAtRPS(ShootingConstants.getRPS(distance));
    } else {
      shooter.runFlywheelAtRPS(tunableRPS.get());
    }

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runFlywheelAtRPS(0);
  }
}
