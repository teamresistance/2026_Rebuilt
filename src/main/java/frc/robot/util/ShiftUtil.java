package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Objects;

public class ShiftUtil {

  private static Constants.ShiftOwner shift1 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift2 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift3 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift4 = Constants.ShiftOwner.BOTH;

  private static final Timer shiftTimer = new Timer();

  public static void setupShifts() {
    String msg = DriverStation.getGameSpecificMessage();
    if (Objects.equals(msg, "B")) {
      shift1 = Constants.ShiftOwner.RED;
      shift2 = Constants.ShiftOwner.BLUE;
      shift3 = Constants.ShiftOwner.RED;
      shift4 = Constants.ShiftOwner.BLUE;
    } else if (Objects.equals(msg, "R")) {
      shift1 = Constants.ShiftOwner.BLUE;
      shift2 = Constants.ShiftOwner.RED;
      shift3 = Constants.ShiftOwner.BLUE;
      shift4 = Constants.ShiftOwner.RED;
    }
    shiftTimer.reset();
    shiftTimer.start();
  }

  /**
   * Returns which alliance owns the <i><b>upcoming</b></i> shift.
   *
   * @return the {@code ShiftOwner} for the <i><b>upcoming</b></i> shift
   */
  public static Constants.ShiftOwner getNextShift() {
    double elapsed = shiftTimer.get();

    if (elapsed < 10.0) {
      return shift1; // transition shift, next is 1
    } else if (elapsed < 35.0) {
      return shift2; // on 1, next is 2
    } else if (elapsed < 60.0) {
      return shift3; // on 2, next is 3
    } else if (elapsed < 85.0) {
      return shift4; // on 3, next is 4
    }

    return Constants.ShiftOwner.BOTH; // next is endgame
  }

  /** Returns if the provided {@code shiftOwner} matches our alliance color. */
  public static boolean isOurs(Constants.ShiftOwner shiftOwner) {
    if (DriverStation.getAlliance().isPresent()) {
      var us = DriverStation.getAlliance().get();
      if (us == DriverStation.Alliance.Red) {
        return (shiftOwner == Constants.ShiftOwner.RED);
      } else if (us == DriverStation.Alliance.Blue) {
        return (shiftOwner == Constants.ShiftOwner.BLUE);
      }
    }
    return true; // just in case!
  }

  /** Returns true if within 5 seconds of the next shift */
  public static boolean nearNextShift() {
    double elapsed = shiftTimer.get();
    if (elapsed < 10.0 && elapsed > 5.0) {
      return true;
    } else if (elapsed < 35.0 && elapsed > 30.0) {
      return true;
    } else if (elapsed < 60.0 && elapsed > 55.0) {
      return true;
    } else return elapsed < 85.0 && elapsed > 80.0;
  }
}
