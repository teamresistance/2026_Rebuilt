package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterIO;

public class TrimCommands {

  private TrimCommands() {
    // Deliberately empty constructor to prevent instantiation of a utility class
  }

  public static Command AdjustVerticalTrim(ShooterIO shooter, double trim) {
    return new InstantCommand(() -> shooter.adjustVerticalTrim(trim));
  }

  public static Command AdjustHorizontalTrim(ShooterIO shooter, double trim) {
    return new InstantCommand(() -> shooter.adjustHorizontalTrim(trim));
  }
}
