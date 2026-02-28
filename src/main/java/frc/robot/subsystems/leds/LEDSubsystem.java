package frc.robot.subsystems.leds;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  private final List<LEDStream> streams = new ArrayList<>();
  private LEDStream lastMode = null;
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

    if (highest != null && !highest.equals(lastMode)) {
      applyMode(highest);
      Logger.recordOutput("LEDS/Active Stream", highest.name);
      lastMode = highest;
    }
  }

  // TODO: find a better way to do the brightness and framerate suppliers in offseason
  private void applyMode(LEDStream mode) {
    Constants.LEDMode ledMode = mode.getLEDMode();
    Logger.recordOutput("LEDS/Active Mode", ledMode);

    switch (ledMode) {
      case RAINBOW:
        if (mode.useFramerateSupplier && mode.useBrightnessSupplier) {
          candle.setControl(
              Constants.LED_ANIMATION_RAINBOW
                  .withBrightness(mode.brightnessSupplier.getAsDouble())
                  .withFrameRate(mode.framerateSupplier.getAsDouble()));
        } else if (mode.useBrightnessSupplier) {
          candle.setControl(
              Constants.LED_ANIMATION_RAINBOW.withBrightness(
                  mode.brightnessSupplier.getAsDouble()));
        } else if (mode.useFramerateSupplier) {
          candle.setControl(
              Constants.LED_ANIMATION_RAINBOW.withFrameRate(mode.framerateSupplier.getAsDouble()));
        } else {
          candle.setControl(Constants.LED_ANIMATION_RAINBOW);
        }
        break;
      case SHOOTING_CONFIDENT:
        candle.setControl(
            Constants.LED_ANIMATION_SHOOTING_CONFIDENT.withFrameRate(
                mode.framerateSupplier.getAsDouble()));
      case SHOOTING_DOUBTFUL:
        candle.setControl(
            Constants.LED_ANIMATION_SHOOTING_DOUBTFUL.withFrameRate(
                mode.framerateSupplier.getAsDouble()));
        break;
      case PASSING_CONFIDENT:
        candle.setControl(
            Constants.LED_ANIMATION_PASSING_CONFIDENT.withFrameRate(
                mode.framerateSupplier.getAsDouble()));
        break;
      case PASSING_DOUBTFUL:
        candle.setControl(
            Constants.LED_ANIMATION_PASSING_DOUBTFUL.withFrameRate(
                mode.framerateSupplier.getAsDouble()));
        break;
      case INTAKING:
        candle.setControl(Constants.LED_ANIMATION_INTAKING);
        break;
      case BUMP:
        candle.setControl(Constants.LED_ANIMATION_BUMP);
        break;
      case AUTO:
        candle.setControl(Constants.LED_ANIMATION_AUTO);
        break;
    }
  }
}
