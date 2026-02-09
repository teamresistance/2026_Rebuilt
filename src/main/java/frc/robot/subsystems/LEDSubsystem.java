package frc.robot.subsystems;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDMode;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  public LEDSubsystem() {}

  private final int LED_START_INDEX = 0;
  private final int LED_END_INDEX = 100;
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
        candle.setControl(new RainbowAnimation(LED_START_INDEX, LED_END_INDEX).withFrameRate(60));
        break;
      case SHOOTING:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(10)
                .withColor(new RGBWColor(100, 255, 100)));
        break;
      case PASSING:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(10)
                .withColor(new RGBWColor(100, 100, 255)));
        break;
      case READY:
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(100, 255, 100)));
        break;
      case NOT_READY:
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 100, 100)));
        break;
      case SHIFTING:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(10)
                .withColor(new RGBWColor(255, 255, 0)));
        break;
      case ENDGAME:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(10)
                .withColor(new RGBWColor(255, 100, 255)));
        break;
      case BUMP:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(10)
                .withColor(new RGBWColor(0, 255, 255)));
        break;
    }
  }
}
