package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;
import java.util.function.BooleanSupplier;

public class HoppertCommand extends Command {

  private static final double PULSE_BACKWARDS_DURATION = 1.4;
  private static final double PULSE_OFF_DURATION = 0;
  private static final double PULSE_FORWARDS_DURATION = 0;

  private static final double HOPPER_WHEELS_DELAY = 0.5;
  private static final double HOPPER_FLOOR_DELAY = 1;

  private static final double OVERCURRENT_THRESHOLD = 80.0;
  private static final double OVERCURRENT_TRIGGER_DURATION = 0.4;
  private static final double REVERSE_DURATION = 0.25;

  private static final double OVERHOPPER_THRESHOLD = 70.0;

  private enum PulseState {
    BACKWARDS,
    OFF,
    FORWARDS
  }

  private final HoppertIO hoppert;
  private final ShooterIO shooter;
  private final IntakeIO intake;

  private final Timer pulseTimer = new Timer();
  private final Timer startTimer = new Timer();
  private final Timer overcurrentTimer = new Timer();
  private final Timer reverseTimer = new Timer();

  private PulseState pulseState = PulseState.BACKWARDS;
  private boolean isOvercurrentTriggered = false;
  private boolean overcurrentTimerRunning = false;

  private BooleanSupplier triggerSup;

  public HoppertCommand(
      HoppertIO hoppert, ShooterIO shooter, IntakeIO intake, BooleanSupplier triggerHeld) {
    this.hoppert = hoppert;
    this.shooter = shooter;
    this.intake = intake;
    triggerSup = triggerHeld;
    addRequirements(hoppert);
  }

  @Override
  public void initialize() {
    reset();
  }

  private void reset() {
    pulseState = PulseState.BACKWARDS;
    pulseTimer.restart();
    startTimer.restart();
    isOvercurrentTriggered = false;
    overcurrentTimerRunning = false;
    overcurrentTimer.stop();
    overcurrentTimer.reset();
    reverseTimer.stop();
    reverseTimer.reset();
  }

  @Override
  public void execute() {

    // Overcurrent detection
    if (hoppert.getMecanumCurrent() > OVERCURRENT_THRESHOLD) {
      //      Logger.recordOutput("Hoppert/JamDetected", true);
      if (!overcurrentTimerRunning) {
        overcurrentTimer.restart();
        overcurrentTimerRunning = true;
      } else if (overcurrentTimer.hasElapsed(OVERCURRENT_TRIGGER_DURATION)) {
        //        Logger.recordOutput("Hoppert/DeJamming", true);
        isOvercurrentTriggered = true;
        reverseTimer.restart();
        overcurrentTimerRunning = false;
        overcurrentTimer.stop();
        overcurrentTimer.reset();
      }
    } else {
      //      Logger.recordOutput("Hoppert/JamDetected", false);
      //      Logger.recordOutput("Hoppert/DeJamming", false);
      overcurrentTimerRunning = false;
      overcurrentTimer.stop();
      overcurrentTimer.reset();
    }

    // Reverse wheels if overcurrent was triggered, until reverse duration elapses
    if (isOvercurrentTriggered) {
      hoppert.reverseHopperWheels();
      if (reverseTimer.hasElapsed(REVERSE_DURATION)) {
        isOvercurrentTriggered = false;
        reverseTimer.stop();
        reverseTimer.reset();
      }
      return;
    }

    if (!shooter.isShooting()) {
      hoppert.stopHopper();
      hoppert.stopWheels();
      if (intake.isIntaking()) {
        hoppert.runHopperBackwardsSlow();
      }
      if (pulseTimer.hasElapsed(PULSE_OFF_DURATION)) {
        pulseState = PulseState.FORWARDS;
        pulseTimer.restart();
      }
      reset();
      return;
    }

    hoppert.runTowerForwards();

    //    switch (pulseState) {
    //      case BACKWARDS -> {
    //        hoppert.runHopperBackwards();
    //        if (pulseTimer.hasElapsed(PULSE_BACKWARDS_DURATION)) {
    //          pulseState = PulseState.OFF;
    //          pulseTimer.restart();
    //        }
    //      }
    //      case OFF -> {
    //        // hoppert.stopHopper();
    //        if (pulseTimer.hasElapsed(PULSE_OFF_DURATION)) {
    //          pulseState = PulseState.FORWARDS;
    //          pulseTimer.restart();
    //        }
    //      }
    //      case FORWARDS -> {
    //        // hoppert.runHopperForwards();
    //        if (pulseTimer.hasElapsed(PULSE_FORWARDS_DURATION)) {
    //          pulseState = PulseState.BACKWARDS;
    //          pulseTimer.restart();
    //        }
    //      }
    //    }

    if (triggerSup.getAsBoolean() && shooter.atShootingSetpoints()) {
      if (startTimer.hasElapsed(HOPPER_WHEELS_DELAY)) {
        hoppert.runHopperWheels();
      }
      if (startTimer.hasElapsed(HOPPER_FLOOR_DELAY)) {
        hoppert.runHopperBackwardsSlow();
      } else {
        hoppert.runHopperBackwards();
      }
    }

    if (hoppert.getHopperCurrent() > OVERHOPPER_THRESHOLD) {
      hoppert.runHopperForwards();
    }
  }
}
