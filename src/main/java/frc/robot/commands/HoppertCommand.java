package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;

public class HoppertCommand extends Command {

  // Set this to 1, 2, or 3
  private static final int MODE = 3;

  // Mode 3 pulse timings (seconds)
  private static final double PULSE_BACKWARDS_DURATION = 1.4;
  private static final double PULSE_OFF_DURATION = 0.3;
  private static final double PULSE_FORWARDS_DURATION = 0;

  private enum PulseState {
    BACKWARDS,
    OFF,
    FORWARDS
  }

  private final HoppertIO hoppert;
  private final ShooterIO shooter;
  private final IntakeIO intake;

  private final Timer pulseTimer = new Timer();
  private PulseState pulseState = PulseState.BACKWARDS;

  public HoppertCommand(HoppertIO hoppert, ShooterIO shooter, IntakeIO intake) {
    this.hoppert = hoppert;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(hoppert);
  }

  @Override
  public void initialize() {
    if (MODE == 3) {
      pulseState = PulseState.BACKWARDS;
      pulseTimer.restart();
    }
  }

  @Override
  public void execute() {
    if (shooter.isShooting()) {
      switch (MODE) {
        case 1 -> runMode1();
        case 2 -> runMode2();
        case 3 -> runMode3();
      }
    } else {
      hoppert.stopHopper();
      hoppert.stopTower();
    }
  }

  // Original behavior
  private void runMode1() {
    hoppert.runHopperBackwards();
    hoppert.runTowerForwards();

    if (shooter.atTargetRPS() && hoppert.towerAtSpeed() && shooter.atShootingSetpoints()) {
      hoppert.runHopperWheels();
    }
  }

  // Hopper forwards instead of backwards
  private void runMode2() {
    hoppert.runHopperForwards();
    hoppert.runTowerForwards();

    if (shooter.atTargetRPS() && hoppert.towerAtSpeed() && shooter.atShootingSetpoints()) {
      hoppert.runHopperWheels();
    }
  }

  // Pulses: backwards -> off -> forwards, timer-driven
  private void runMode3() {

    hoppert.runHopperWheels();
    hoppert.runTowerForwards();

    switch (pulseState) {
      case BACKWARDS -> {
        hoppert.runHopperBackwards();
        if (pulseTimer.hasElapsed(PULSE_BACKWARDS_DURATION)) {
          pulseState = PulseState.OFF;
          pulseTimer.restart();
        }
      }
      case OFF -> {
        hoppert.stopHopper();
        if (pulseTimer.hasElapsed(PULSE_OFF_DURATION)) {
          pulseState = PulseState.FORWARDS;
          pulseTimer.restart();
        }
      }
      case FORWARDS -> {
        hoppert.runHopperForwards();
        if (pulseTimer.hasElapsed(PULSE_FORWARDS_DURATION)) {
          pulseState = PulseState.BACKWARDS;
          pulseTimer.restart();
        }
      }
    }
  }
}
