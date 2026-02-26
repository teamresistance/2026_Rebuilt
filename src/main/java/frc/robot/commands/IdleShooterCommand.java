package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    // NOTE: All angles are field-relative!
    double distance =
        ShooterIO
            .getPredictedDistanceToHubAfterReload(); // Use predicted distance after reload for idle
    // aiming
    double turretAngle = ShooterIO.getHorizontalTotalShootingAngle();
    double hoodAngle = ShooterIO.getVerticalShootingAngle();

    shooter.setTurretTarget(turretAngle);
    shooter.setHoodTarget(hoodAngle);

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
  }
}
