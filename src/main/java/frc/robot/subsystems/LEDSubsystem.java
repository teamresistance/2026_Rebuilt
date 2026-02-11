package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDMode;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  public LEDSubsystem() {}

  private final CANdle candle = new CANdle(60);

  private LEDMode mode = LEDMode.RAINBOW;
  private boolean isLocked = false;

  public void setMode(LEDMode newMode, boolean lock) {
    if (!isLocked) mode = newMode;
    if (lock) isLocked = true;
  }

  public void unlock() {
    isLocked = false;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("LED Mode", mode.toString());

    // auto set led
    switch (mode) {
      case RAINBOW:
        candle.setControl(Constants.LED_ANIMATION_RAINBOW);
        break;
      case SHOOTING:
        candle.setControl(Constants.LED_ANIMATION_SHOOTING);
        break;
      case PASSING:
        candle.setControl(Constants.LED_ANIMATION_PASSING);
        break;
      case READY:
        candle.setControl(Constants.LED_ANIMATION_READY);
        break;
      case NOT_READY:
        candle.setControl(Constants.LED_ANIMATION_NOT_READY);
        break;
      case SHIFTING:
        candle.setControl(Constants.LED_ANIMATION_SHIFTING);
        break;
      case ENDGAME:
        candle.setControl(Constants.LED_ANIMATION_ENDGAME);
        break;
      case BUMP:
        candle.setControl(Constants.LED_ANIMATION_BUMP);
        break;
    }
  }
}
