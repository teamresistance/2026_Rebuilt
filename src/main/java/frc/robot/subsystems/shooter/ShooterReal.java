package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.ShootingUtil;
import frc.robot.util.ShootingUtil.Conversions;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {

  private final TalonFX hoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID, CANBus.roboRIO());
  private final TalonFX turretMotor = new TalonFX(Constants.SHOOTER_TURRET_ID, CANBus.roboRIO());
  private final TalonFX flywheelMotor =
      new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, CANBus.roboRIO());
  private final TalonFX flywheelMotor2 =
      new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_2, CANBus.roboRIO());
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

    // TODO: tune me and get rid of the ultra slow starting values
    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(1).withKI(0).withKD(0).withKS(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(250)
                    .withMotionMagicCruiseVelocity(10))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true));
    hoodMotor.getConfigurator().apply(hoodConfig);

    // TODO: tune me and get rid of the ultra slow starting values
    TalonFXConfiguration turretConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(1).withKI(0).withKD(0).withKS(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(50)
                    .withMotionMagicCruiseVelocity(10))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs().withRemoteCANcoder(turretEncoder).withRotorToSensorRatio(20));
    turretMotor.getConfigurator().apply(turretConfig);

    TalonFXConfiguration flywheelConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(100).withKI(0).withKD(0).withKS(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true));
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    TalonFXConfiguration flywheelConfig2 =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(0)
                    .withSupplyCurrentLimitEnable(true));
    flywheelMotor2.getConfigurator().apply(flywheelConfig2);
  }

  /**
   * Returns {@code true} when the turret hood and turret base are at their setpoints, with a
   * tolerance of {@code Constants.SHOOTER_HOOD_REVS_TOLERANCE} and {@code
   * Constants.SHOOTER_TURRET_REVS_TOLERANCE}, respectively.
   */
  @Override
  public boolean atShootingSetpoints() {
    return hoodMotor.getPosition().isNear(hoodTargetAngle, Constants.SHOOTER_HOOD_REVS_TOLERANCE)
        && turretMotor
            .getPosition()
            .isNear(turretTargetAngle, Constants.SHOOTER_TURRET_REVS_TOLERANCE);
  }

  /** Returns if the flywheel motors are at their target speed. */
  @Override
  public boolean atTargetRPS() {
    return flywheelMotor.getVelocity().isNear(flywheelTargetRPS, Constants.SHOOTER_RPS_TOLERANCE);
  }

  /** Sets the velocity target for the flywheel in rotations/second */
  @Override
  public void runFlywheelAtRPS(double rps) {
    flywheelTargetRPS = rps;
    flywheelMotor.setControl(new VelocityDutyCycle(rps));
    flywheelMotor2.setControl(new StrictFollower(Constants.SHOOTER_FLYWHEEL_ID));
  }

  /**
   * Sets the turret hood's target (in degrees). Must be within {@code
   * Constants.SHOOTER_HOOD_MIN_PITCH} and {@code Constants.SHOOTER_HOOD_MAX_PITCH}
   */
  @Override
  public void setHoodTarget(double angle) {
    if (angle < Constants.SHOOTER_HOOD_MAX_PITCH && angle > Constants.SHOOTER_HOOD_MIN_PITCH) {
      angle = angle - Constants.SHOOTER_HOOD_MIN_PITCH; // zero position is not zero degrees
      hoodTargetAngle = angle;
      hoodMotor.setControl(new MotionMagicVoltage(ShootingUtil.toHoodRevs(angle)));
    }
  }

  /**
   * Sets the turret base's target (in degrees). Must be within {@code
   * Constants.SHOOTER_TURRET_MIN_YAW} and {@code Constants.SHOOTER_TURRET_MAX_YAW}
   */
  @Override
  public void setTurretTarget(double angle) {
    if (angle < Constants.SHOOTER_TURRET_MAX_YAW && angle > Constants.SHOOTER_TURRET_MIN_YAW) {
      turretTargetAngle = angle;
      turretMotor.setControl(new MotionMagicVoltage(ShootingUtil.toTurretRevs(angle)));
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
      hoodMotor.setPosition(Conversions.toHoodRevs(newValueDegrees));
    }
  }

  @Override
  public boolean isShooting() {
    return flywheelTargetRPS > 1.0;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Flywheel Target RPS", flywheelTargetRPS);
    Logger.recordOutput(
        "Shooter/Flywheel Real RPS", flywheelMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/Turret Target Angle", turretTargetAngle);
    Logger.recordOutput(
        "Shooter/Turret Real Angle",
        Conversions.toTurretDegrees(turretEncoder.getPosition().getValueAsDouble()));
    Logger.recordOutput("Shooter/Hood Target Angle", hoodTargetAngle);
    Logger.recordOutput(
        "Shooter/Hood Real Angle",
        Conversions.toHoodDegrees(hoodMotor.getPosition().getValueAsDouble()));
  }
}
