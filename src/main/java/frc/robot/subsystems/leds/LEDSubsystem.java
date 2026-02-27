package frc.robot.subsystems.leds;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
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
      lastMode = highest;
    }
  }

  // TODO: find a better way to do the brightness and framerate suppliers in offseason
  private void applyMode(LEDStream mode) {
    Constants.LEDMode ledMode = mode.getLEDMode();
    Logger.recordOutput("LED Mode", ledMode);

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
      case SHOOTING:
        if (mode.useFramerateSupplier) {
          candle.setControl(
              Constants.LED_ANIMATION_SHOOTING.withFrameRate(mode.framerateSupplier.getAsDouble()));
        } else {
          candle.setControl(Constants.LED_ANIMATION_SHOOTING);
        }
        break;
      case PASSING:
        if (mode.useFramerateSupplier) {
          candle.setControl(
              Constants.LED_ANIMATION_PASSING.withFrameRate(mode.framerateSupplier.getAsDouble()));
        } else {
          candle.setControl(Constants.LED_ANIMATION_PASSING);
        }
        break;
      case READY:
        if (mode.useFramerateSupplier) {
          double brightness = mode.brightnessSupplier.getAsDouble();
          RGBWColor modified =
              new RGBWColor(
                  (int) (Constants.LED_ANIMATION_READY.Color.Red * brightness),
                  (int) (Constants.LED_ANIMATION_READY.Color.Green * brightness),
                  (int) (Constants.LED_ANIMATION_READY.Color.Blue * brightness));
          candle.setControl(Constants.LED_ANIMATION_READY.withColor(modified));
        } else {
          candle.setControl(Constants.LED_ANIMATION_READY);
        }
        break;
      case NOT_READY:
        if (mode.useFramerateSupplier) {
          double brightness = mode.brightnessSupplier.getAsDouble();
          RGBWColor modified =
              new RGBWColor(
                  (int) (Constants.LED_ANIMATION_NOT_READY.Color.Red * brightness),
                  (int) (Constants.LED_ANIMATION_NOT_READY.Color.Green * brightness),
                  (int) (Constants.LED_ANIMATION_NOT_READY.Color.Blue * brightness));
          candle.setControl(Constants.LED_ANIMATION_NOT_READY.withColor(modified));
        } else {
          candle.setControl(Constants.LED_ANIMATION_NOT_READY);
        }
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
      case AUTO:
        candle.setControl(Constants.LED_ANIMATION_AUTO);
        break;
    }
  }
}
