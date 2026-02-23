package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class IntakeSim implements IntakeIO {

  private boolean intaking = false;
  private boolean rejecting = false;

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Intaking", intaking);
    Logger.recordOutput("Intake/Rejecting", rejecting);
  }

  @Override
  public void activateIntake() {
    intaking = true;
    rejecting = false;
  }

  @Override
  public void stopIntake() {
    intaking = false;
    rejecting = false;
  }

  @Override
  public void reverseIntake() {
    rejecting = true;
    intaking = false;
  }

  @Override
  public boolean isIntaking() {
    return intaking;
  }
}
