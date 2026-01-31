package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingParameters;
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
    double[] shootingParams =
        ShootingUtil.getAngleToAim(drive.getPose(), drive.getChassisSpeeds(), 1);
    int roundedScaledDistance = (int) (shootingParams[1] * 10);
    shooter.setTurretTarget(shootingParams[0]);
    shooter.setHoodTarget(ShootingParameters.params[roundedScaledDistance][1]);
    shooter.runFlywheelAtRPS(ShootingParameters.params[roundedScaledDistance][0]);
    // TODO: Control the feeder based on the RPS being good/bad and the turret and hood being ready
  }
}
