package frc.robot.subsystems.hoppert;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** The Hoppert is the hopper and tower combined. */
public interface HoppertIO extends Subsystem {

  @Override
  void periodic();

  void runHopperBackwards();

  double getMecanumCurrent();

  void runHopperForwards();

  void runTowerBackwards();

  void runTowerForwards();

  boolean towerAtSpeed();

  void runHopperWheels();

  void reverseHopperWheels();

  void stopHopper();

  void stopTower();
}
