package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeReal implements IntakeIO {

  private final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, CANBus.roboRIO());
  private boolean intaking = false;
  private boolean rejecting = false;

  public IntakeReal() {
    register();

    // TODO: current limits
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(20));

    intakeMotor.getConfigurator().apply(config);
  }

  @Override
  public void activateIntake() {
    intaking = true;
    rejecting = false;
    intakeMotor.setControl(new DutyCycleOut(0.5));
  }

  @Override
  public void reverseIntake() {
    rejecting = true;
    intaking = false;
    intakeMotor.set(-1);
  }

  @Override
  public void stopIntake() {
    intaking = false;
    rejecting = false;
    intakeMotor.setControl(new CoastOut());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Intaking", intaking);
    Logger.recordOutput("Intake/Rejecting", rejecting);
  }

  @Override
  public boolean isIntaking() {
    return intaking;
  }
}
