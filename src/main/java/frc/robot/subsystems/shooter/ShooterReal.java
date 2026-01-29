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

  // 13 - full down TODO: correct hard stop
  // 47 - full up TODO: correct hard stop
  private double hoodTargetAngle = 0;

  // 180 - straight forward
  // 10 - full left TODO: correct left hard stop
  // 350 - full right TODO: correct right hard stop
  private double turretTargetAngle = 180;

  private double flywheelTargetRPS = 0;

  /** Real implementation of a turret shooter. */
  public ShooterReal() {
    register();
    configure();
  }

  @Override
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
                // otherwise, correct the ratio if it is mounted elsewhere
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

  @Override
  public boolean atShootingSetpoints() {
    return hoodMotor.getPosition().isNear(hoodTargetAngle, Constants.SHOOTER_HOOD_REVS_TOLERANCE)
        && turretEncoder
            .getPosition()
            .isNear(turretTargetAngle, Constants.SHOOTER_TURRET_REVS_TOLERANCE);
  }

  @Override
  public void runFlywheelAtRPS(double rps) {
    flywheelTargetRPS = rps;
    flywheelMotor.setControl(new VelocityDutyCycle(rps));
  }

  @Override
  public void setHoodTarget(double angle) {
    hoodTargetAngle = angle;
    flywheelMotor.setControl(new PositionDutyCycle(ShootingUtil.toHoodRevs(angle)));
  }

  @Override
  public void setTurretTarget(double angle) {
    turretTargetAngle = angle;
    turretMotor.setControl(new PositionDutyCycle(ShootingUtil.toTurretRevs(angle)));
  }

  @Override
  public void zeroHood(double newValue) {
    hoodMotor.setPosition(newValue);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Flywheel Target RPS", flywheelTargetRPS);
    Logger.recordOutput(
        "Shooter/Flywheel Real RPS", flywheelMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/Turret Target Angle", turretTargetAngle);
    Logger.recordOutput(
        "Shooter/Turret Real Angle",
        ShootingUtil.toTurretDegrees(ShootingUtil.toTurretDegrees(turretEncoder.getPosition().getValueAsDouble())));
    Logger.recordOutput("Shooter/Hood Target Angle", hoodTargetAngle);
    Logger.recordOutput(
      "Shooter/Hood Real Angle",
      ShootingUtil.toTurretDegrees(ShootingUtil.toHoodDegrees(hoodMotor.getPosition().getValueAsDouble())));
  }
}
