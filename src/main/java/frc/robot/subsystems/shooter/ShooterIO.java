package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {

  @Override
  void periodic();

  boolean atShootingSetpoints();

  void runFlywheelAtRPS(double rps);

  void setHoodTarget(double hoodTargetAngle);

  void setTurretTarget(double turretTargetAngle, double omegaRadsPerSec);

  boolean atTargetRPS();

  boolean isShooting();

  void zeroHood(double newValue);

  double getDriveAssistanceAngle();

  void setSwivelStop(boolean stopped);

  void adjustVerticalTrim(boolean up);

  void adjustHorizontalTrim(boolean right);
}
