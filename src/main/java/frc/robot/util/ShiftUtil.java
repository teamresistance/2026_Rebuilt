package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class ShiftUtil {

  private static Constants.ShiftOwner shift1 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift2 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift3 = Constants.ShiftOwner.BOTH;
  private static Constants.ShiftOwner shift4 = Constants.ShiftOwner.BOTH;

  private static boolean wasAssigned = false;
  private static final Timer shiftTimer = new Timer();

  /** Must be called at the beginning of teleop to sync the shifts with the match clock */
  public static void startShiftTimer() {
    shiftTimer.reset();
    shiftTimer.start();
  }

  public static String getShiftColor() {
    Constants.ShiftOwner currentShift = getShift();

    return switch (currentShift) {
      case RED -> "#FF0000"; // Red
      case BLUE -> "#0000FF"; // Blue
      case BOTH -> "#FF00FF"; // Purple
    };
  }

  public static String getShiftColor(Constants.ShiftOwner currentShift) {

    return switch (currentShift) {
      case RED -> "#FF0000"; // Red
      case BLUE -> "#0000FF"; // Blue
      case BOTH -> "#FF00FF"; // Purple
    };
  }

  /**
   * Automatically assigns shifts based on FMS game data. Called after a 1-second delay to ensure
   * the game-specific message has been sent
   */
  public static void assignShifts() {
    String msg = DriverStation.getGameSpecificMessage();
    if (Objects.equals(msg, "B")) {
      shift1 = Constants.ShiftOwner.RED;
      shift2 = Constants.ShiftOwner.BLUE;
      shift3 = Constants.ShiftOwner.RED;
      shift4 = Constants.ShiftOwner.BLUE;
      wasAssigned = true;
    } else if (Objects.equals(msg, "R")) {
      shift1 = Constants.ShiftOwner.BLUE;
      shift2 = Constants.ShiftOwner.RED;
      shift3 = Constants.ShiftOwner.BLUE;
      shift4 = Constants.ShiftOwner.RED;
      wasAssigned = true;
    }
    Logger.recordOutput("Shifts/Automatically assigned", wasAssigned);
    Logger.recordOutput("Shifts/Shift 1", shift1);
    Logger.recordOutput("Shifts/Shift 2", shift2);
    Logger.recordOutput("Shifts/Shift 3", shift3);
    Logger.recordOutput("Shifts/Shift 4", shift4);
  }

  /** Manually assigns shifts based on whoever was active first. */
  public static void assignShifts(String msg) {
    if (Objects.equals(msg, "R")) {
      shift1 = Constants.ShiftOwner.RED;
      shift2 = Constants.ShiftOwner.BLUE;
      shift3 = Constants.ShiftOwner.RED;
      shift4 = Constants.ShiftOwner.BLUE;
      wasAssigned = true;
    } else if (Objects.equals(msg, "B")) {
      shift1 = Constants.ShiftOwner.BLUE;
      shift2 = Constants.ShiftOwner.RED;
      shift3 = Constants.ShiftOwner.BLUE;
      shift4 = Constants.ShiftOwner.RED;
      wasAssigned = true;
    }
    Logger.recordOutput("Shifts/Shift 1", shift1);
    Logger.recordOutput("Shifts/Shift 2", shift2);
    Logger.recordOutput("Shifts/Shift 3", shift3);
    Logger.recordOutput("Shifts/Shift 4", shift4);
  }

  public static boolean isAssigned() {
    return wasAssigned;
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

  /**
   * Returns which alliance owns this shift.
   *
   * @return the {@code ShiftOwner} for this shift
   */
  public static Constants.ShiftOwner getShift() {
    double elapsed = shiftTimer.get();

    if (elapsed < 10.0) {
      return Constants.ShiftOwner.BOTH; // transition shift
    } else if (elapsed < 35.0) {
      return shift1;
    } else if (elapsed < 60.0) {
      return shift2;
    } else if (elapsed < 85.0) {
      return shift3;
    } else if (elapsed < 110.0) {
      return shift4;
    }

    return Constants.ShiftOwner.BOTH; // endgame
  }

  /** Returns the number of seconds left in the current shift. Returns 0 after endgame. */
  public static double getTimeLeftInCurrentShift() {
    double elapsed = shiftTimer.get();
    double remaining;

    if (elapsed < 10.0) {
      remaining = 10.0 - elapsed; // transition -> next boundary at 10
    } else if (elapsed < 35.0) {
      remaining = 35.0 - elapsed; // shift 1 -> next at 35
    } else if (elapsed < 60.0) {
      remaining = 60.0 - elapsed; // shift 2 -> next at 60
    } else if (elapsed < 85.0) {
      remaining = 85.0 - elapsed; // shift 3 -> next at 85
    } else if (elapsed < 110.0) {
      remaining = 110.0 - elapsed; // shift 4 -> next at 110
    } else {
      remaining = 0.0; // endgame or beyond
    }

    // Clamp to zero to avoid tiny negatives from timing imprecision
    return Math.max(0.0, remaining);
  }

  public static boolean isOurs(Constants.ShiftOwner shiftOwner) {
    var alliance = DriverStation.getAlliance();
    // fallback
    return alliance
        .map(
            value ->
                switch (value) {
                  case Red ->
                      shiftOwner == Constants.ShiftOwner.RED
                          || shiftOwner == Constants.ShiftOwner.BOTH;
                  case Blue ->
                      shiftOwner == Constants.ShiftOwner.BLUE
                          || shiftOwner == Constants.ShiftOwner.BOTH;
                })
        .orElse(true);
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

  /** Returns true if within 7 seconds of the next shift */
  public static boolean withinSevenSecondsOfNextShift() {
    double elapsed = shiftTimer.get();
    if (elapsed < 10.0 && elapsed > 3.0) {
      return true;
    } else if (elapsed < 35.0 && elapsed > 28.0) {
      return true;
    } else if (elapsed < 60.0 && elapsed > 53.0) {
      return true;
    } else return elapsed < 85.0 && elapsed > 78.0;
  }

  /** Returns true if within 2 seconds of the next shift */
  public static boolean withinTwoSecondsOfNextShift() {
    double elapsed = shiftTimer.get();
    if (elapsed < 10.0 && elapsed > 8.0) {
      return true;
    } else if (elapsed < 35.0 && elapsed > 33.0) {
      return true;
    } else if (elapsed < 60.0 && elapsed > 58.0) {
      return true;
    } else return elapsed < 85.0 && elapsed > 83.0;
  }

  /** Returns true if we are currently in the endgame phase (after 85 seconds + 20s) */
  public static boolean isDeepEndgame() {
    return shiftTimer.get() >= 85.0 + 20;
  }

  public static String getAutoWinnerColor() {
    return ShiftUtil.getShiftColor(shift2);
  }
}
