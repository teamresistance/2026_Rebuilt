package frc.robot.subsystems.leds;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  private final List<LEDStream> streams = new ArrayList<>();
  private Constants.LEDMode lastMode = Constants.LEDMode.RAINBOW;
  private final CANdle candle = new CANdle(60);

  public LEDSubsystem() {}

  /** Adds an LEDStream to the list of periodically checked streams. */
  public void addStream(LEDStream stream) {
    streams.add(stream);
  }

  @Override
  public void periodic() {

    LEDStream highest = null;

    for (LEDStream stream : streams) {

      if (!stream.isActive()) continue;

      if (highest == null || stream.priority >= highest.priority) {
        highest = stream;
      }
    }

    Constants.LEDMode newMode =
        (highest == null) ? Constants.LEDMode.RAINBOW : highest.getLEDMode();

    if (!newMode.equals(lastMode)) {
      applyMode(newMode);
      lastMode = newMode;
    }
  }

  private void applyMode(Constants.LEDMode mode) {
    Logger.recordOutput("LED Mode", mode.toString());

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
      case SHIFTING_US:
        candle.setControl(Constants.LED_ANIMATION_SHIFTING_US);
        break;
      case SHIFTING_THEM:
        candle.setControl(Constants.LED_ANIMATION_SHIFTING_THEM);
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
