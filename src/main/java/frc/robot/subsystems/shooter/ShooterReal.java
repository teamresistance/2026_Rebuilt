package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {

  private final TalonFX hoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID, CANBus.roboRIO());
  private final TalonFX turretMotor = new TalonFX(Constants.SHOOTER_TURRET_ID, CANBus.roboRIO());
  private final TalonFX flywheelMotor =
      new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, CANBus.roboRIO());
  private final CANcoder turretEncoder =
      new CANcoder(Constants.SHOOTER_TURRET_ENCODER_ID, CANBus.roboRIO());

  private double hoodTargetAngle = 0;
  private double turretTargetAngle = 180;
  private double flywheelTargetRPS = 0;

  /** Real implementation of a turret shooter. */
  public ShooterReal() {
    register();
    configure();
  }

  /** Configures motors (control mode, pid, current limits) */
  public void configure() {

    // TODO: motion magic or pid, for all

    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true));
    hoodMotor.getConfigurator().apply(hoodConfig);

    TalonFXConfiguration turretConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                // using CANcoder and assuming encoder mounted directly to motor.
                // TODO: otherwise, correct the ratio if it is mounted elsewhere
                new FeedbackConfigs().withRemoteCANcoder(turretEncoder).withRotorToSensorRatio(1));
    turretMotor.getConfigurator().apply(turretConfig);

    TalonFXConfiguration flywheelConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true));
    flywheelMotor.getConfigurator().apply(flywheelConfig);
  }

  /**
   * Returns {@code true} when the turret hood and turret base are at their setpoints, with a
   * tolerance of {@code Constants.SHOOTER_HOOD_REVS_TOLERANCE} and {@code
   * Constants.SHOOTER_TURRET_REVS_TOLERANCE}, respectively.
   */
  @Override
  public boolean atShootingSetpoints() {
    return hoodMotor.getPosition().isNear(hoodTargetAngle, Constants.SHOOTER_HOOD_REVS_TOLERANCE)
        && turretEncoder
            .getPosition()
            .isNear(turretTargetAngle, Constants.SHOOTER_TURRET_REVS_TOLERANCE);
  }

  /** Sets the velocity target for the flywheel in rotations/second */
  @Override
  public void runFlywheelAtRPS(double rps) {
    flywheelTargetRPS = rps;
    flywheelMotor.setControl(new VelocityDutyCycle(rps));
  }

  /**
   * Sets the turret hood's target (in degrees). Must be within {@code
   * Constants.SHOOTER_HOOD_MIN_PITCH} and {@code Constants.SHOOTER_HOOD_MAX_PITCH}
   */
  @Override
  public void setHoodTarget(double angle) {
    if (angle > Constants.SHOOTER_HOOD_MAX_PITCH || angle < Constants.SHOOTER_HOOD_MIN_PITCH) {
      hoodTargetAngle = angle;
      flywheelMotor.setControl(new PositionDutyCycle(ShootingUtil.toHoodRevs(angle)));
    }
  }

  /**
   * Sets the turret base's target (in degrees). Must be within {@code
   * Constants.SHOOTER_TURRET_MIN_YAW} and {@code Constants.SHOOTER_TURRET_MAX_YAW}
   */
  @Override
  public void setTurretTarget(double angle) {
    if (angle > Constants.SHOOTER_TURRET_MAX_YAW || angle < Constants.SHOOTER_TURRET_MIN_YAW) {
      turretTargetAngle = angle;
      turretMotor.setControl(new PositionDutyCycle(ShootingUtil.toTurretRevs(angle)));
    }
  }

  /**
   * Sets the hood's sensor position to {@code newValueDegrees}. Must be within {@code
   * Constants.SHOOTER_HOOD_MIN_PITCH} and {@code Constants.SHOOTER_HOOD_MAX_PITCH}
   */
  @Override
  public void zeroHood(double newValueDegrees) {
    if (newValueDegrees > Constants.SHOOTER_HOOD_MAX_PITCH
        || newValueDegrees < Constants.SHOOTER_HOOD_MIN_PITCH) {
      hoodMotor.setPosition(ShootingUtil.toHoodRevs(newValueDegrees));
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Flywheel Target RPS", flywheelTargetRPS);
    Logger.recordOutput(
        "Shooter/Flywheel Real RPS", flywheelMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/Turret Target Angle", turretTargetAngle);
    Logger.recordOutput(
        "Shooter/Turret Real Angle",
        ShootingUtil.toTurretDegrees(
            ShootingUtil.toTurretDegrees(turretEncoder.getPosition().getValueAsDouble())));
    Logger.recordOutput("Shooter/Hood Target Angle", hoodTargetAngle);
    Logger.recordOutput(
        "Shooter/Hood Real Angle",
        ShootingUtil.toTurretDegrees(
            ShootingUtil.toHoodDegrees(hoodMotor.getPosition().getValueAsDouble())));
  }
}
