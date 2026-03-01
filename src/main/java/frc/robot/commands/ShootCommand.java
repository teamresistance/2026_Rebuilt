package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
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

    shooter.runFlywheelAtRPS(Units.radiansToRotations(ShooterIO.getDesiredAngularVelocity()));

    Logger.recordOutput("Shooter/Desired Angular Velocity", ShooterIO.getDesiredAngularVelocity());
  }
}
