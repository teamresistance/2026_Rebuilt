package frc.robot.subsystems.hoppert;

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
    //    Logger.recordOutput("Hoppert/Rollers Active", hopperRollersRunning);
    //    Logger.recordOutput("Hoppert/Rollers Reversed", hopperRollersReversed);
    //    Logger.recordOutput("Hoppert/Wheels Active", hopperWheelsRunning);
    //    Logger.recordOutput("Hoppert/Tower Motor Active", towerMotorRunning);
    //    Logger.recordOutput("Hoppert/Tower Motor Reversed", towerMotorReversed);
  }

  @Override
  public void runHopperBackwards() {
    hopperRollersRunning = true;
    hopperRollersReversed = true;
  }

  @Override
  public double getHopperCurrent() {
    return 0;
  }

  @Override
  public double getMecanumCurrent() {
    return 0;
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
  public boolean towerAtSpeed() {
    return true; // i cba to do this
  }

  @Override
  public void runHopperWheels() {
    hopperWheelsRunning = true;
  }

  @Override
  public void reverseHopperWheels() {
    hopperWheelsRunning = true;
  }

  @Override
  public void stopHopper() {
    hopperRollersRunning = false;
    hopperWheelsRunning = false;
  }

  @Override
  public void stopWheels() {
    // idc
  }

  @Override
  public void stopTower() {
    towerMotorRunning = false;
  }
}
