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
