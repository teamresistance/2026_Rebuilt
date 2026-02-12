package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ShiftUtilTest {

  @BeforeEach
  public void setup() {
    assertTrue(HAL.initialize(500, 0));

    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setGameSpecificMessage("B");
    DriverStationSim.notifyNewData();

    SimHooks.restartTiming();
    ShiftUtil.setupShifts();
  }

  @Test
  @DisplayName("getNextShift correct for all shifts")
  public void testShiftOwnershipSequence() {

    // 0s → next shift is shift1 → RED
    assertEquals(Constants.ShiftOwner.RED, ShiftUtil.getNextShift());

    // Advance to 12s → next shift is shift2 → BLUE
    SimHooks.stepTiming(12.0);
    assertEquals(Constants.ShiftOwner.BLUE, ShiftUtil.getNextShift());

    // Advance to 40s → next shift is shift3 → RED
    SimHooks.stepTiming(28.0);
    assertEquals(Constants.ShiftOwner.RED, ShiftUtil.getNextShift());

    // Advance to 70s → next shift is shift4 → BLUE
    SimHooks.stepTiming(30.0);
    assertEquals(Constants.ShiftOwner.BLUE, ShiftUtil.getNextShift());

    // Advance past 85s → endgame → BOTH
    SimHooks.stepTiming(20.0);
    assertEquals(Constants.ShiftOwner.BOTH, ShiftUtil.getNextShift());
  }

  @Test
  @DisplayName("nearNextShift works as intended in 5s range")
  public void testNearNextShiftWindows() {

    // Not near at 0s
    assertFalse(ShiftUtil.nearNextShift());

    // Near first shift (5-10s window)
    SimHooks.stepTiming(6.0);
    assertTrue(ShiftUtil.nearNextShift());

    // Not near
    SimHooks.stepTiming(10.0);
    assertFalse(ShiftUtil.nearNextShift());

    // Near second shift window (30-35s)
    SimHooks.stepTiming(15.0);
    assertTrue(ShiftUtil.nearNextShift());
  }

  @Test
  @DisplayName("isOurs correctly identifies if a shift is our alliance color")
  public void testIsOurs() {

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    assertTrue(ShiftUtil.isOurs(Constants.ShiftOwner.RED));
    assertFalse(ShiftUtil.isOurs(Constants.ShiftOwner.BLUE));
  }

  @Test
  @DisplayName("setupShifts works correctly with both game-specific messages")
  public void testGameSpecificMessageR() {

    DriverStationSim.setGameSpecificMessage("R");
    DriverStationSim.notifyNewData();

    ShiftUtil.setupShifts();

    assertEquals(Constants.ShiftOwner.BLUE, ShiftUtil.getNextShift());
  }
}
