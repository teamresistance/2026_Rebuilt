package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

public class ClimberSim implements ClimberIO {

  // TODO: actual sim logic once more details known

  private boolean braking = false;
  private boolean isUp = true;
  private DCMotor climber = DCMotor.getNeo550(1).withReduction(64);

  @Override
  public void brake() {
    braking = true;
  }

  @Override
  public void unbrake() {
    braking = false;
  }

  @Override
  public void up() {
    isUp = true;
  }

  @Override
  public void down() {
    isUp = false;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Setpoint", isUp);
    Logger.recordOutput("Climber/Position", isUp);
    Logger.recordOutput("Climber/Braking", braking);
  }

}
