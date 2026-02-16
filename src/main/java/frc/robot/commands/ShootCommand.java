package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.util.shooter.ShootingManager;
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

    shooter.runFlywheelAtRPS(Units.radiansToRotations(ShootingManager.getDesiredAngularVelocity()));

    Logger.recordOutput("Shooter/Desired Angular Velocity", ShootingManager.getDesiredAngularVelocity());

    // TODO: all of these
    //
    // It was initially said: Control the feeder based on the RPS being good/bad and the turret and hood being ready
    // However: maximum RPS is calculated to be about 100 rad/s, about 955 RPM, which << 5000 RPM, the maximum sustained flywheel speed.
    //
    // It was initially said: if the virtual pose is FAR out of the alliance zone (aka the balls will NOT make it
    //                        in under any circumstances), some way to stop shooting.
    // However: the virtual pose represents the pose after about 0.07s. At a maximum of 5 ft/s, the robot only moves about 10cm.
  }
}
