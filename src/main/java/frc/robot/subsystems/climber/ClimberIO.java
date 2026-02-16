package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClimberIO extends Subsystem {

  @Override
  void periodic();

  void brake();

  void unbrake();

  void up();

  void down();
}
