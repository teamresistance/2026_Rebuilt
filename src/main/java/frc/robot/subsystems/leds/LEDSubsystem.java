package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;

public class LEDSubsystem extends SubsystemBase {

  private final List<LEDStream> streams = new ArrayList<>();
  private Constants.LEDMode lastLEDMode = null;
  private final CANdle candle = new CANdle(Constants.CANDLE_ID, new CANBus("drive"));

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

    if (highest != null) {
      Constants.LEDMode currentMode = highest.getLEDMode();
      // Only apply mode if it changed to avoid repeatedly calling setControl()
      // if (lastLEDMode != currentMode) {
      applyMode(currentMode);
      lastLEDMode = currentMode;
      // }
      //      Logger.recordOutput("LEDS/Active Stream", highest.name);
    }
  }

  private void applyMode(Constants.LEDMode ledMode) {
    // Logger.recordOutput("LEDS/Active Mode", ledMode);

    switch (ledMode) {
      case DISABLED:
        double voltage = RobotController.getBatteryVoltage();
        if (voltage >= 12.6) {
          candle.setControl(Constants.LED_ANIMATION_DISABLED_GOOD);
        } else if (voltage > 12.2) {
          candle.setControl(Constants.LED_ANIMATION_DISABLED_FINE);
        } else {
          candle.setControl(Constants.LED_ANIMATION_DISABLED_BAD);
        }
        break;
      case RAINBOW:
        candle.setControl(Constants.LED_ANIMATION_RAINBOW);
        break;
      case SHOOTING_CONFIDENT:
        candle.setControl(Constants.LED_ANIMATION_SHOOTING_CONFIDENT);
        // .withFrameRate(
        //        mode.framerateSupplier.getAsDouble()));
        break;
      //      case SHOOTING_DOUBTFUL:
      //        candle.setControl(
      //            Constants.LED_ANIMATION_SHOOTING_DOUBTFUL.withFrameRate(
      //                mode.framerateSupplier.getAsDouble()));
      //        break;
      //      case PASSING_CONFIDENT:
      //        candle.setControl(
      //            Constants.LED_ANIMATION_PASSING_CONFIDENT.withFrameRate(
      //                mode.framerateSupplier.getAsDouble()));
      //        break;
      //      case PASSING_DOUBTFUL:
      //        candle.setControl(
      //            Constants.LED_ANIMATION_PASSING_DOUBTFUL.withFrameRate(
      //                mode.framerateSupplier.getAsDouble()));
      //        break;
      //      case INTAKING:
      //        candle.setControl(Constants.LED_ANIMATION_INTAKING);
      //        break;
      case BUMP:
        candle.setControl(Constants.LED_ANIMATION_BUMP);
        break;
      //      case AUTO:
      //        candle.setControl(Constants.LED_ANIMATION_AUTO);
      //        break;
      case ACTIVE:
        candle.setControl(Constants.LED_ANIMATION_ACTIVE);
        break;
      case INACTIVE:
        candle.setControl(Constants.LED_ANIMATION_INACTIVE);
        break;
      case CLOSE_TO_NEXT_SHIFT:
        candle.setControl(Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT);
        break;
      case CLOSE_TO_NEXT_SHIFT_US:
        candle.setControl(Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_US);
        break;
      case CLOSE_TO_NEXT_SHIFT_NOTUS:
        candle.setControl(Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_NOTUS);
        break;
      case ENDGAME:
        candle.setControl(Constants.LED_ANIMATION_ENDGAME);
        break;
    }
  }
}
