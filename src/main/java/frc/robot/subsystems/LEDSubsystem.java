package frc.robot.subsystems;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  // hardware
  private final CANdle candle = new CANdle(Constants.LED_CANDLE_ID, Constants.LED_CANDLE_BUS);
  private final int LED_START_INDEX = 0;
  private final int LED_END_INDEX = 151;

  // tracking
  private Constants.LED_MODE ledMode = Constants.LED_MODE.DISABLED_DIM;

  // animations
  private final SolidColor ANIM_ACTIVE = new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(0, 255, 0));
  private final SolidColor ANIM_INACTIVE = new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 0, 0));
  private final StrobeAnimation ANIM_SHIFT_WARNING = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 165, 0)).withFrameRate(4);
  private final StrobeAnimation ANIM_SHIFT_SWITCHING_RED = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 0, 0)).withFrameRate(8);
  private final StrobeAnimation ANIM_SHIFT_SWITCHING_GREEN = ANIM_SHIFT_SWITCHING_RED.withColor(new RGBWColor(0, 0, 255));
  private final StrobeAnimation ANIM_ENDGAME_WARNING = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(200, 0, 200)).withFrameRate(4);
  private final StrobeAnimation ANIM_CLIMB_WARNING = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(200, 0, 200)).withFrameRate(10);
  private final StrobeAnimation ANIM_AUTO = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(0, 255, 0)).withFrameRate(4);
  private final StrobeAnimation ANIM_DISABLED_DIM = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(25, 0, 0)).withFrameRate(2);

  public void setModeActive() {
    candle.setControl(ANIM_ACTIVE);
    ledMode = Constants.LED_MODE.ACTIVE;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeInactive() {
    candle.setControl(ANIM_INACTIVE);
    ledMode = Constants.LED_MODE.INACTIVE;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeShiftWarning() {
    candle.setControl(ANIM_SHIFT_WARNING);
    ledMode = Constants.LED_MODE.SHIFT_WARNING;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeShiftSwitchingRed() {
    candle.setControl(ANIM_SHIFT_SWITCHING_RED);
    ledMode = Constants.LED_MODE.SHIFT_SWITCHING_RED;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeShiftSwitchingGreen() {
    candle.setControl(ANIM_SHIFT_SWITCHING_GREEN);
    ledMode = Constants.LED_MODE.SHIFT_SWITCHING_GREEN;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeEndgameWarning() {
    candle.setControl(ANIM_ENDGAME_WARNING);
    ledMode = Constants.LED_MODE.ENDGAME_WARNING;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeClimbWarning() {
    candle.setControl(ANIM_CLIMB_WARNING);
    ledMode = Constants.LED_MODE.CLIMB_WARNING;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeAuto() {
    candle.setControl(ANIM_AUTO);
    ledMode = Constants.LED_MODE.AUTO;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  public void setModeDisabledDim() {
    candle.setControl(ANIM_DISABLED_DIM);
    ledMode = Constants.LED_MODE.DISABLED_DIM;
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDS/Active", ledMode);
  }
}
