package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.util.ShiftUtil;

public class ContinuousLEDCommand extends Command {

  private final LEDSubsystem leds;
  private final SwerveDriveIO drive;

  /**
   * This command should always be running. All logic for deciding the LED mode should occur within
   * this command - therefore any subsystems that affect LEDs must be passed through this
   * constructor along with the LED subsystem.
   */
  public ContinuousLEDCommand(LEDSubsystem leds, SwerveDriveIO drive) {
    this.leds = leds;
    this.drive = drive;
    addRequirements(leds);
  }

  @Override
  public void execute() {
    if (DriverStation.isDisabled()) {
      leds.setModeDisabledDim();
      return;
    }
    if (DriverStation.isAutonomous()) {
      leds.setModeAuto();
      return;
    }
    if (ShiftUtil.isVeryDeepEndgame()) {
      leds.setModeClimbWarning();
      return;
    }
    if (Constants.tooClose) {
      leds.setModeTooClose();
      return;
    }
    if (ShiftUtil.isDeepEndgame()) {
      leds.setModeEndgameWarning();
      return;
    }
    if (ShiftUtil.isOurs(ShiftUtil.getShift())) { // we are active
      if (ShiftUtil.withinTwoSecondsOfNextShift()) {
        if (ShiftUtil.isOurs(ShiftUtil.getNextShift())) { // next shift is also ours
          leds.setModeShiftSwitchingGreen();
        } else { // next shift is theirs
          leds.setModeShiftSwitchingRed();
        }
        return;
      }
      if (ShiftUtil.withinSevenSecondsOfNextShift()) {
        leds.setModeShiftWarning();
        return;
      }
      leds.setModeActive();
    } else { // not active
      if (ShiftUtil.withinTwoSecondsOfNextShift()) { // not active, 2s of next, next must be us
        leds.setModeShiftSwitchingGreen();
        return;
      }
      if (ShiftUtil.withinSevenSecondsOfNextShift()) {
        leds.setModeShiftWarning();
        return;
      }
      leds.setModeInactive();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
