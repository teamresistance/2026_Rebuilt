package frc.robot.subsystems.hoppert;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HoppertReal implements HoppertIO {

  private final TalonFX hopperRollerMotor =
      new TalonFX(Constants.HOPPER_ROLLERS_ID, CANBus.roboRIO());
  private final TalonFX hopperWheelsMotor =
      new TalonFX(Constants.HOPPER_WHEELS_ID, CANBus.roboRIO());
  private final TalonFX towerMotor = new TalonFX(Constants.TOWER_MOTOR_ID, CANBus.roboRIO());

  private boolean hopperRollersRunning = false;
  private boolean hopperWheelsRunning = false;
  private boolean towerMotorRunning = false;

  private boolean towerMotorReversed = false;
  private boolean hopperRollersReversed = false;

  public HoppertReal() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    towerMotor.getConfigurator().apply(config);

    register();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hoppert/Rollers Active", hopperRollersRunning);
    Logger.recordOutput("Hoppert/Rollers Reversed", hopperRollersReversed);
    Logger.recordOutput("Hoppert/Wheels Active", hopperWheelsRunning);
    Logger.recordOutput("Hoppert/Tower Motor Active", towerMotorRunning);
    Logger.recordOutput("Hoppert/Tower Motor Reversed", towerMotorReversed);
  }

  @Override
  public void runHopperBackwards() {
    hopperRollerMotor.setControl(new DutyCycleOut(-1.0));
    hopperRollersRunning = true;
    hopperRollersReversed = true;
  }

  @Override
  public void runHopperForwards() {
    hopperRollerMotor.setControl(new DutyCycleOut(1.0));
    hopperRollersRunning = true;
    hopperRollersReversed = false;
  }

  @Override
  public void runTowerBackwards() {
    towerMotor.setControl(new DutyCycleOut(-1.0));
    towerMotorRunning = true;
    towerMotorReversed = true;
  }

  @Override
  public void runTowerForwards() {
    towerMotor.setControl(new DutyCycleOut(1.0));
    towerMotorRunning = true;
    towerMotorReversed = false;
  }

  @Override
  public void runHopperWheels() {
    hopperWheelsMotor.setControl(new DutyCycleOut(1.0));
    hopperWheelsRunning = true;
  }

  @Override
  public void stopHopper() {
    hopperRollerMotor.setControl(new CoastOut());
    hopperWheelsMotor.setControl(new CoastOut());
    hopperRollersRunning = false;
    hopperWheelsRunning = false;
  }

  @Override
  public void stopTower() {
    towerMotor.setControl(new StaticBrake());
    hopperRollersRunning = false;
  }
}
