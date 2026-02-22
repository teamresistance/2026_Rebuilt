package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeReal implements IntakeIO {

  private final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, CANBus.roboRIO());
  private boolean intakeActive = false;
  private boolean isRejecting = false;

  public IntakeReal() {
    register();

    // TODO: current limits
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0));

    intakeMotor.getConfigurator().apply(config);
  }

  @Override
  public void activateIntake() {
    intakeActive = true;
    isRejecting = false;
    intakeMotor.setControl(new DutyCycleOut(1));
  }

  @Override
  public void reverseIntake() {
    isRejecting = true;
    intakeActive = false;
    intakeMotor.set(-1);
  }

  @Override
  public void stopIntake() {
    intakeActive = false;
    isRejecting = false;
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Intaking", intakeActive);
    Logger.recordOutput("Intake/Rejecting", isRejecting);
  }

  @Override
  public boolean isIntaking() {
    return intakeActive;
  }
}
