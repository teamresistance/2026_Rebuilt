package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.ShootingUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {

  private final TalonFX hoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID, CANBus.roboRIO());
  private final TalonFX turretMotor = new TalonFX(Constants.SHOOTER_TURRET_ID, CANBus.roboRIO());
  private final TalonFX flywheelMotor =
      new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, CANBus.roboRIO());
  private final TalonFX flywheelMotor2 =
      new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_2, CANBus.roboRIO());

  private TalonFXConfiguration turretConfig;
  private TalonFXConfiguration hoodConfig;

  private double hoodTargetAngle = 0;
  private double turretTargetAngle = 0;
  private double turretDriveAssistTargetAngle = 0;
  private double flywheelTargetRPS = 0;
  private double verticalTrim = 0.0;
  private double horizontalTrim = 0.0;

  private boolean emergencyStopSwivel = false;

  /** Real implementation of a turret shooter. */
  public ShooterReal() {
    register();
    configure();
    zeroHood(Constants.SHOOTER_HOOD_MIN_PITCH);
  }

  /** Configures motors (control mode, pid, current limits) */
  public void configure() {

    hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(300)
                    .withMotionMagicCruiseVelocity(70))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(20)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(20)
                    .withSupplyCurrentLimitEnable(true));
    hoodMotor.getConfigurator().apply(hoodConfig);

    turretConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(2).withKI(0).withKD(0).withKS(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(600)
                    .withMotionMagicCruiseVelocity(130)
                    .withMotionMagicJerk(3000))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitThreshold(
                        ShootingUtil.toTurretRevs(Constants.SHOOTER_TURRET_MIN_YAW))
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        ShootingUtil.toTurretRevs(Constants.SHOOTER_TURRET_MAX_YAW))
                    .withForwardSoftLimitEnable(true));
    turretMotor.getConfigurator().apply(turretConfig);

    TalonFXConfiguration flywheelConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(12).withKI(0).withKD(0).withKS(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withPeakReverseDutyCycle(0));
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    TalonFXConfiguration flywheelConfig2 =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    flywheelMotor2.getConfigurator().apply(flywheelConfig2);
  }

  /**
   * Returns {@code true} when the turret hood and turret base are at their setpoints, with a
   * tolerance of {@code Constants.SHOOTER_HOOD_REVS_TOLERANCE} and {@code
   * Constants.SHOOTER_TURRET_REVS_TOLERANCE}, respectively.
   */
  @Override
  public boolean atShootingSetpoints() {
    return hoodMotor
            .getPosition()
            .isNear(ShootingUtil.toHoodRevs(hoodTargetAngle), Constants.SHOOTER_HOOD_REVS_TOLERANCE)
        && turretMotor
            .getPosition()
            .isNear(
                ShootingUtil.toTurretRevs(turretTargetAngle),
                Constants.SHOOTER_TURRET_REVS_TOLERANCE);
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
    if (rps == 0) {
      flywheelMotor.setControl(new CoastOut());
      flywheelMotor2.setControl(new CoastOut());
      return;
    }
    flywheelMotor.setControl(new VelocityDutyCycle(rps));
    flywheelMotor2.setControl(new StrictFollower(Constants.SHOOTER_FLYWHEEL_ID));
  }

  /**
   * Sets the turret hood's target (in degrees). Must be within {@code
   * Constants.SHOOTER_HOOD_MIN_PITCH} and {@code Constants.SHOOTER_HOOD_MAX_PITCH}
   */
  @Override
  public void setHoodTarget(double angle) {
    angle += verticalTrim; // add vertical trim to hood target angle
    if (angle < Constants.SHOOTER_HOOD_MAX_PITCH && angle > Constants.SHOOTER_HOOD_MIN_PITCH) {
      angle = angle - Constants.SHOOTER_HOOD_MIN_PITCH; // zero position is not zero degrees
      hoodTargetAngle = angle;
      hoodMotor.setControl(
          new MotionMagicVoltage(ShootingUtil.toHoodRevs(angle)).withEnableFOC(true));
    }
  }

  /**
   * Sets the turret base's target (in degrees). Must be within {@code
   * Constants.SHOOTER_TURRET_MIN_YAW} and {@code Constants.SHOOTER_TURRET_MAX_YAW}
   */
  @Override
  public void setTurretTarget(double angle, double omegaRadsPerSec) {
    if (emergencyStopSwivel) {
      // stop swivel, aim with drive, offset from stopped position
      turretDriveAssistTargetAngle =
          angle + (ShootingUtil.toTurretDegrees(turretMotor.getPosition().getValueAsDouble()));
    } else {
      double turretAngle =
          MathUtil.clamp(angle, Constants.SHOOTER_TURRET_MIN_YAW, Constants.SHOOTER_TURRET_MAX_YAW);
      if (Math.abs(angle - turretAngle) > 2.0) {
        turretDriveAssistTargetAngle = MathUtil.inputModulus(angle - turretAngle, -180, 180);
      } else {
        turretDriveAssistTargetAngle = 0;
      }
      turretTargetAngle =
          turretAngle + horizontalTrim; // add horizontal trim to turret target angle
      turretMotor.setControl(
          new MotionMagicVoltage(ShootingUtil.toTurretRevs(turretTargetAngle))
              .withEnableFOC(true)
              .withFeedForward(-omegaRadsPerSec * 2));
    }
  }

  /**
   * Sets the hood's sensor position to {@code newValueDegrees}. Must be within {@code
   * Constants.SHOOTER_HOOD_MIN_PITCH} and {@code Constants.SHOOTER_HOOD_MAX_PITCH}
   */
  @Override
  public void zeroHood(double newValueDegrees) {
    if (newValueDegrees < Constants.SHOOTER_HOOD_MAX_PITCH
        && newValueDegrees > Constants.SHOOTER_HOOD_MIN_PITCH) {
      hoodMotor.setPosition(ShootingUtil.toHoodRevs(newValueDegrees));
    }
  }

  @Override
  public double getDriveAssistanceAngle() {
    return turretDriveAssistTargetAngle;
  }

  @Override
  public void setSwivelStop(boolean stopped) {
    emergencyStopSwivel = stopped;
  }

  @Override
  public boolean isShooting() {
    return flywheelTargetRPS >= 1.0;
  }

  @Override
  public void adjustVerticalTrim(boolean up) {
    if (up) {
      verticalTrim += Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else {
      verticalTrim -= Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    }
  }

  @Override
  public void adjustHorizontalTrim(boolean right) {
    if (right) {
      horizontalTrim += Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else {
      horizontalTrim -= Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    }
  }

  @Override
  public void brake() {
    turretMotor
        .getConfigurator()
        .apply(
            hoodConfig.withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));
    //    hoodMotor
    //        .getConfigurator()
    //        .apply(
    //            turretConfig.withMotorOutput(
    //                new MotorOutputConfigs()
    //                    .withNeutralMode(NeutralModeValue.Brake)
    //                    .withInverted(InvertedValue.Clockwise_Positive)));
  }

  @Override
  public void coast() {
    turretMotor
        .getConfigurator()
        .apply(
            hoodConfig.withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
    //    hoodMotor
    //        .getConfigurator()
    //        .apply(
    //            turretConfig.withMotorOutput(
    //                new MotorOutputConfigs()
    //                    .withNeutralMode(NeutralModeValue.Coast)
    //                    .withInverted(InvertedValue.Clockwise_Positive)));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Flywheel Target RPS", flywheelTargetRPS);
    Logger.recordOutput(
        "Shooter/Flywheel Real RPS", flywheelMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/Turret Target Angle", turretTargetAngle);
    Logger.recordOutput(
        "Shooter/Turret Real Angle",
        ShootingUtil.toTurretDegrees(turretMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput("Shooter/Hood Target Angle", hoodTargetAngle);
    Logger.recordOutput(
        "Shooter/Hood Real Angle",
        ShootingUtil.toTurretDegrees(
            ShootingUtil.toHoodDegrees(hoodMotor.getPosition().getValueAsDouble())));
    Logger.recordOutput("Shooter/Drive Assist Angle", turretDriveAssistTargetAngle);
    Logger.recordOutput("Shooter/Vertical Trim", verticalTrim);
    Logger.recordOutput("Shooter/Horizontal Trim", horizontalTrim);
    Logger.recordOutput("Shooter/AtSetpoints HT", atShootingSetpoints());
    Logger.recordOutput("Shooter/AtSetpoints F", atTargetRPS());
  }
}
