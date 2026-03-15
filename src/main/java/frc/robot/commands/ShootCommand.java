package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingStyle;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShootingMaps;
import frc.robot.subsystems.shooter.ShootingPredictions;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class ShootCommand extends Command {

  private final ShooterIO shooter;
  private final SwerveDriveIO drive;
  private final ShootingStyle calcMode;

  public ShootCommand(SwerveDriveIO _drive, ShooterIO _shooter, ShootingStyle _calcMode) {
    shooter = _shooter;
    drive = _drive;
    calcMode = _calcMode;
  }

  @Override
  public void execute() {
    if (calcMode == ShootingStyle.MAPS) {
      double distance =
          ShootingUtil.getVirtualDistanceToTarget(drive.getPose(), drive.getChassisSpeeds());
      shooter.runFlywheelAtRPS(ShootingMaps.getRPS(distance));
      Logger.recordOutput("Shooter/Desired Angular Velocity", ShootingMaps.getRPS(distance));

    } else if (calcMode == ShootingStyle.CALC) {
      shooter.runFlywheelAtRPS(ShootingPredictions.getCalculator().getDesiredAngularVelocity());
      Logger.recordOutput(
          "Shooter/Desired Angular Velocity",
          ShootingPredictions.getCalculator().getDesiredAngularVelocity());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runFlywheelAtRPS(0);
  }
}
