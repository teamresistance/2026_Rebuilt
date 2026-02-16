package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberReal implements ClimberIO {

  private final SparkMax climber = new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder climberInternalEncoder = climber.getEncoder();
  private final SparkClosedLoopController climberControl = climber.getClosedLoopController();

  private final Relay brakeSolenoid = new Relay(Constants.CLIMBER_BRAKE_ID);

  public ClimberReal() {
    register();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(0).i(0).d(0); // TODO: tune me! tune me!
    climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void brake() {
    brakeSolenoid.set(Relay.Value.kOff);
  }

  @Override
  public void unbrake() {
    brakeSolenoid.set(Relay.Value.kOn);
  }

  @Override
  public void up() {
    climberControl.setSetpoint(Constants.CLIMBER_FULL, SparkBase.ControlType.kPosition);
  }

  @Override
  public void down() {
    climberControl.setSetpoint(Constants.CLIMBER_ZERO, SparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Setpoint", climberControl.getSetpoint());
    Logger.recordOutput("Climber/Position", climberInternalEncoder.getPosition());
    Logger.recordOutput("Climber/Braking", brakeSolenoid.get().equals(Relay.Value.kOff));
  }
}
