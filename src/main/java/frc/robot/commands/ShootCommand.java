package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.util.ShootingUtil;

public class ShootCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public ShootCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
  }

  @Override
  public void execute() {
    shooter.runFlywheelAtRPS(
        ShootingConstants.getRPS(
            ShootingUtil.getVirtualDistanceToHub( // used for maximum accuracy
                drive.getPose(), drive.getChassisSpeedsFieldRelative())));
    // TODO: Control the feeder based on the RPS being good/bad and the turret and hood being ready
  }
}
