package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {

  @Override
  void periodic();

  boolean atShootingSetpoints();

  void runFlywheelAtRPS(double rps);

  // TODO: In implementation, the angles here must be field-relative.
  void setHoodTarget(double hoodTargetAngle);

  // TODO: In implementation, the angles here must be field-relative.
  void setTurretTarget(double turretTargetAngle);

  void zeroHood(double newValue);
}
