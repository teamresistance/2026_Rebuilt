package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {

  /**
   * This is the instance of {@code ShootingPredictions} that should be referenced by other shooter
   * classes.
   */
  static ShootingPredictions calculator = new ShootingPredictions();

  static ShootingPredictions getCalculator() {
    return calculator;
  }

  @Override
  void periodic();

  boolean atShootingSetpoints();

  void runFlywheelAtRPS(double rps);

  void setHoodTarget(double hoodTargetAngle);

  void setTurretTarget(double turretTargetAngle);

  boolean atTargetRPS();

  boolean isShooting();

  void zeroHood(double newValue);
}
