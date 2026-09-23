package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIO;
import java.util.function.BooleanSupplier;

public class IntakeCommand extends Command {

  private static final double JAM_CURRENT_THRESHOLD = 70.0;
  private static final double JAM_DETECT_DURATION = 0.3;
  private static final double REVERSE_DURATION = 0.3;

  private final IntakeIO intake;
  private final BooleanSupplier intaking;
  private final BooleanSupplier shouldReverse;

  private double highCurrentTimer = 0.0;
  private double reverseTimer = 0.0;
  private boolean isReversing = false;

  public IntakeCommand(
      IntakeIO intake, BooleanSupplier shouldIntake, BooleanSupplier shouldReverse) {
    this.intake = intake;
    this.intaking = shouldIntake;
    this.shouldReverse = shouldReverse;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    highCurrentTimer = 0.0;
    reverseTimer = 0.0;
    isReversing = false;
  }

  @Override
  public void execute() {

    // auto handles intake thru toggles and this will break it
    if (DriverStation.isAutonomous()) return;

    if (shouldReverse.getAsBoolean()) {
      intake.reverseIntake();
      return;
    }

    double dt = 0.02;

    if (isReversing) {
      reverseTimer += dt;
      if (reverseTimer >= REVERSE_DURATION) {
        isReversing = false;
        reverseTimer = 0.0;
        highCurrentTimer = 0.0;
      } else {
        intake.reverseIntake();
      }
      return;
    }

    if (intaking.getAsBoolean()) {
      // check for jam
      if (intake.getIntakeCurrent() > JAM_CURRENT_THRESHOLD) {
        highCurrentTimer += dt;
        if (highCurrentTimer >= JAM_DETECT_DURATION) {
          isReversing = true;
          reverseTimer = 0.0;
          intake.reverseIntake();
          return;
        }
      } else {
        highCurrentTimer = 0.0; // reset when current drops
      }

      intake.activateIntake();
    } else {
      highCurrentTimer = 0.0;
      intake.stopIntake();
    }
  }
}
