package frc.robot.subsystems.hoppert;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** The Hoppert is the hopper and tower combined. */
public interface HoppertIO extends Subsystem {

  @Override
  void periodic();

  void runHopperBackwards();

  void runHopperForwards();

  void runTowerBackwards();

  void runTowerForwards();

  void runHopperWheels();

  void stopHopper();

  void stopTower();
}
