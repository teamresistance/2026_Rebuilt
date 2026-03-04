package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {

  /**
   * This is the instance of {@code ShootingPredictions} that should be referenced by other shooter classes.
   */
  static ShootingPredictions calculator = new ShootingPredictions();

  @Override
  public void periodic();

  public boolean atShootingSetpoints();

  public void runFlywheelAtRPS(double rps);

  public void setHoodTarget(double hoodTargetAngle);

  public void setTurretTarget(double turretTargetAngle);

  public boolean atTargetRPS();

  public boolean isShooting();

  void zeroHood(double newValue);
}
