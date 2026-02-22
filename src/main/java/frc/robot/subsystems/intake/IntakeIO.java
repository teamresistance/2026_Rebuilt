package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem {

  @Override
  void periodic();

  void activateIntake();

  void stopIntake();

  void reverseIntake();

  boolean isIntaking();
}
