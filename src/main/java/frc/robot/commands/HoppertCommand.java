package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;

public class HoppertCommand extends Command {

  private final HoppertIO hoppert;
  private final ShooterIO shooter;
  private final IntakeIO intake;

  public HoppertCommand(HoppertIO hoppert, ShooterIO shooter, IntakeIO intake) {
    this.hoppert = hoppert;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(hoppert);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean isShooting = shooter.isShooting();

    if (isShooting) {
      hoppert.runHopperBackwards();
      hoppert.runTowerForwards();

      if (shooter.atTargetRPS() && hoppert.towerAtSpeed()) {
        hoppert.runHopperWheels();
      }
    } else {
      hoppert.stopHopper();
      hoppert.stopTower();
    }
  }
}
