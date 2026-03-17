package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;

public class HoppertCommand extends Command {

  private final HoppertIO hoppert;
  private final ShooterIO shooter;
  private final IntakeIO intake;
  private final Timer shootingTimer = new Timer();
  private boolean wasShooting = false;

  public HoppertCommand(HoppertIO hoppert, ShooterIO shooter, IntakeIO intake) {
    this.hoppert = hoppert;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(hoppert);
  }

  @Override
  public void initialize() {
    shootingTimer.stop();
    shootingTimer.reset();
    wasShooting = false;
  }

  @Override
  public void execute() {
    if (shooter.isShooting()) {

      if (!wasShooting) {
        shootingTimer.restart();
      }
      wasShooting = true;

      hoppert.runTowerForwards();

      if (shootingTimer.hasElapsed(0.25)) {
        hoppert.runHopperBackwards();
      }

      if (shooter.atTargetRPS() && hoppert.towerAtSpeed() && shooter.atShootingSetpoints()) {
        hoppert.runHopperWheels();
      }
    } else {
      wasShooting = false;
      shootingTimer.stop();
      shootingTimer.reset();
      hoppert.stopHopper();
      hoppert.stopTower();
    }
  }
}
