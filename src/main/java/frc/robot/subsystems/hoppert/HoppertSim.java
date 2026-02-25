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

public class HoppertSim implements HoppertIO {

  private boolean hopperRollersRunning = false;
  private boolean hopperWheelsRunning = false;
  private boolean towerMotorRunning = false;

  private boolean towerMotorReversed = false;
  private boolean hopperRollersReversed = false;

  public HoppertSim() {
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
    hopperRollersRunning = true;
    hopperRollersReversed = true;
  }

  @Override
  public void runHopperForwards() {
    hopperRollersRunning = true;
    hopperRollersReversed = false;
  }

  @Override
  public void runTowerBackwards() {
    towerMotorRunning = true;
    towerMotorReversed = true;
  }

  @Override
  public void runTowerForwards() {
    towerMotorRunning = true;
    towerMotorReversed = false;
  }

  @Override
  public void runHopperWheels() {
    hopperWheelsRunning = true;
  }

  @Override
  public void stopHopper() {
    hopperRollersRunning = false;
  }

  @Override
  public void stopTower() {
    hopperRollersRunning = false;
  }
}
