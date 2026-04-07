package frc.robot.subsystems.hoppert;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class HoppertReal implements HoppertIO {

  private final TalonFX hopperRollerMotor =
      new TalonFX(Constants.HOPPER_ROLLERS_ID, CANBus.roboRIO());
  private final TalonFX hopperWheelsMotor =
      new TalonFX(Constants.HOPPER_WHEELS_ID, CANBus.roboRIO());
  private final TalonFX towerMotor = new TalonFX(Constants.TOWER_MOTOR_ID, CANBus.roboRIO());

  private final LoggedTunableNumber hopperFloorRPS =
      new LoggedTunableNumber("Hoppert/TunableHopperFloorSpeed", -24);

  private boolean hopperRollersRunning = false;
  private boolean hopperWheelsRunning = false;
  private boolean towerMotorRunning = false;

  private boolean towerMotorReversed = false;
  private boolean hopperRollersReversed = false;

  public HoppertReal() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(12));
    towerMotor.getConfigurator().apply(config);

    TalonFXConfiguration config2 =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(1));
    hopperRollerMotor.getConfigurator().apply(config2);

    TalonFXConfiguration config3 =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(6));
    hopperWheelsMotor.getConfigurator().apply(config3);

    register();
  }

  @Override
  public void periodic() {
    //    Logger.recordOutput("Hoppert/Rollers Active", hopperRollersRunning);
    //    Logger.recordOutput("Hoppert/Wheels Active", hopperWheelsRunning);
    //    Logger.recordOutput("Hoppert/Tower Motor Active", towerMotorRunning);
    //    Logger.recordOutput("Hoppert/TowerSpeed", towerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Hoppert/TowerTemp", towerMotor.getDeviceTemp().getValueAsDouble());
    //    Logger.recordOutput("Hoppert/TowerCurrent",
    // towerMotor.getSupplyCurrent().getValueAsDouble());
    //    Logger.recordOutput(
    //        "Hoppert/BottomCurrent", hopperRollerMotor.getSupplyCurrent().getValueAsDouble());
  }

  @Override
  public double getMecanumCurrent() {
    return hopperWheelsMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void runHopperBackwards() {
    hopperRollerMotor.setControl(new VelocityVoltage(hopperFloorRPS.get()));
    hopperRollersRunning = true;
    hopperRollersReversed = true;
  }

  @Override
  public void runHopperForwards() {
    hopperRollerMotor.setControl(new VelocityVoltage(4));
    hopperRollersRunning = true;
    hopperRollersReversed = false;
  }

  @Override
  public void runTowerBackwards() {
    towerMotor.setControl(new VelocityVoltage(58).withEnableFOC(true));
    towerMotorRunning = true;
    towerMotorReversed = true;
  }

  @Override
  public void runTowerForwards() {
    towerMotor.setControl(new VelocityVoltage(-58).withEnableFOC(true));
    towerMotorRunning = true;
    towerMotorReversed = false;
  }

  @Override
  public boolean towerAtSpeed() {
    return towerMotor.getVelocity().isNear(-58, 8);
  }

  @Override
  public void runHopperWheels() {
    hopperWheelsMotor.setControl(new VelocityVoltage(150));
    hopperWheelsRunning = true;
  }

  @Override
  public void reverseHopperWheels() {
    hopperWheelsMotor.setControl(new VelocityVoltage(-60));
    hopperWheelsRunning = true;
  }

  @Override
  public void stopHopper() {
    hopperRollerMotor.setControl(new CoastOut());
    hopperRollersRunning = false;
  }

  @Override
  public void stopWheels() {
    hopperWheelsMotor.setControl(new StaticBrake());
    hopperWheelsRunning = false;
  }

  @Override
  public void stopTower() {
    towerMotor.setControl(new CoastOut());
    towerMotorRunning = false;
  }
}
