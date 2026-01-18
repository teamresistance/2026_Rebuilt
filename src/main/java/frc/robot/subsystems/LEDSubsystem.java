package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    SmartDashboard.putString("LED Mode", mode.toString());
    Logger.recordOutput("LED Mode", mode.toString());

    // auto set led
    switch (mode) {
        // do stuff
    }
  }
}
