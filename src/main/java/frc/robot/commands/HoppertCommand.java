package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.shooter.ShooterIO;

public class HoppertCommand extends Command {

  private final HoppertIO hoppert;
  private final ShooterIO shooter;
  private final Timer directionSwitchTimer = new Timer();

  // TODO: add intake to this
  public HoppertCommand(HoppertIO hoppert, ShooterIO shooter) {
    this.hoppert = hoppert;
    this.shooter = shooter;
    addRequirements(hoppert);
  }

  @Override
  public void initialize() {
    directionSwitchTimer.reset();
  }

  @Override
  public void execute() {

    // TODO: only run the hopper when at target speed too? test if motors stall constantly or
    // something

    if (shooter.isShooting()) {

      // wheels always running when shooting
      hoppert.runHopperWheels();

      // every 4s, switch to running it forwards and restart the timer
      // forwards-ness lasts for 1 second
      if (directionSwitchTimer.hasElapsed(4)) {
        hoppert.runHopperForwards();
        directionSwitchTimer.reset();
      } else if (directionSwitchTimer.hasElapsed(1)) {
        hoppert.runHopperBackwards();
      }

      // only feed through tower when flywheel is ready
      if (shooter.atTargetRPS()) {
        hoppert.runTowerForwards();
      }
    } else {
      // do not run stuff when not shooting
      hoppert.stopHopper();
      hoppert.stopTower();
    }
  }
}
