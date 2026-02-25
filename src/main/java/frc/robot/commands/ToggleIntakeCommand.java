package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIO;

public class ToggleIntakeCommand extends Command {

  private final IntakeIO intake;

  public ToggleIntakeCommand(IntakeIO intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    if (intake.isIntaking()) {
      intake.stopIntake();
    } else {
      intake.activateIntake();
    }
  }
}
