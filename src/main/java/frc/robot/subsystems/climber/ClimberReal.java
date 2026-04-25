package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.Constants;

public class ClimberReal implements ClimberIO {

  private final SparkMax climber = new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final SparkClosedLoopController climberControl = climber.getClosedLoopController();

  private double trim = 0;

  private final Relay brakeRelay = new Relay(Constants.CLIMBER_BRAKE_RELAY_ID);

  public ClimberReal() {
    register();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(1).i(0).d(0);
    config.closedLoop.allowedClosedLoopError(1, ClosedLoopSlot.kSlot0);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void brake() {
    brakeRelay.set(Relay.Value.kOff);
  }

  @Override
  public void unbrake() {
    brakeRelay.set(Relay.Value.kReverse);
  }

  @Override
  public void up() {
    climberControl.setSetpoint(
        Constants.CLIMBER_FULL_OUT, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void down() {
    climberControl.setSetpoint(
        Constants.CLIMBER_FULL_IN + trim, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public boolean atTarget() {
    return climberControl.isAtSetpoint();
  }

  @Override
  public void trimDown() {
    climberControl.setSetpoint(
        Constants.CLIMBER_FULL_IN + trim, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    trim--;
  }

  @Override
  public void periodic() {
    //    Logger.recordOutput("Climber/Setpoint", climberControl.getSetpoint());
    //    Logger.recordOutput("Climber/Position", climber.getEncoder().getPosition());
    //    Logger.recordOutput("Climber/Braking", brakeRelay.get() == Relay.Value.kOff);
  }
}
