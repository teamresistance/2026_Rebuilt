package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class IntakeSim implements IntakeIO {

  private boolean intakeActive = false;
  private boolean isRejecting = false;

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Intaking", intakeActive);
    Logger.recordOutput("Intake/Rejecting", isRejecting);
  }

  @Override
  public void activateIntake() {
    intakeActive = true;
    isRejecting = false;
  }

  @Override
  public void stopIntake() {
    intakeActive = false;
    isRejecting = false;
  }

  @Override
  public void reverseIntake() {
    isRejecting = true;
    intakeActive = false;
  }

  @Override
  public boolean isIntaking() {
    return intakeActive;
  }
}
