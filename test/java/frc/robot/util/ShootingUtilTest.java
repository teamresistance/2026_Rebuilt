package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.util.ShootingUtil.BallisticSolution;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ShootingUtilTest {

  // Tolerance in meters for landing position assertions.
  // Matches the 0.05-0.08m error budget discussed in design.
  private static final double POSITION_TOLERANCE_M = 0.05;

  // Tolerance in m/s for launch speed assertions.
  private static final double SPEED_TOLERANCE_MPS = 0.1;

  // Tolerance in degrees for angle assertions.
  private static final double ANGLE_TOLERANCE_DEG = 1.5;

  @BeforeEach
  public void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  // ---------------------------------------------------------------------------
  // computeBallistics (no acceleration overload)
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] Stationary robot: solution lands within position tolerance at 4m")
  public void testStationaryShot_4m() {
    double D = 4.0;
    double azimuthDeg = 0.0;

    BallisticSolution sol = ShootingUtil.computeBallistics(D, azimuthDeg, 0.0, 0.0);

    // Back-simulate to verify landing position
    double[] landing =
        ShootingUtil.predictLandingPose(
            0,
            0,
            sol.launchSpeed()
                * Math.cos(Math.toRadians(sol.hoodAngleDeg()))
                * Math.cos(Math.toRadians(sol.deltaAzimuthDeg())),
            sol.launchSpeed()
                * Math.cos(Math.toRadians(sol.hoodAngleDeg()))
                * Math.sin(Math.toRadians(sol.deltaAzimuthDeg())),
            sol.launchSpeed() * Math.sin(Math.toRadians(sol.hoodAngleDeg())));

    assertEquals(
        D,
        landing[0],
        POSITION_TOLERANCE_M,
        "[Calculated Approach] X landing position should be within tolerance of target distance");
    assertEquals(
        0.0,
        landing[1],
        POSITION_TOLERANCE_M,
        "[Calculated Approach] Y landing position should be within tolerance of zero lateral error");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Stationary robot: solution is physically plausible (positive speed, angle)")
  public void testStationaryShot_plausibility() {
    BallisticSolution sol = ShootingUtil.computeBallistics(4.0, 0.0, 0.0, 0.0);

    assertTrue(sol.launchSpeed() > 0, "[Calculated Approach] Launch speed must be positive");
    assertTrue(
        sol.hoodAngleDeg() > 0 && sol.hoodAngleDeg() < 90,
        "[Calculated Approach] Hood angle must be between 0 and 90 degrees");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Stationary robot: azimuth correction is zero when aiming directly at target")
  public void testStationaryShot_noAzimuthCorrection() {
    // No robot velocity means Galilean correction is zero — azimuth delta should be near zero
    BallisticSolution sol = ShootingUtil.computeBallistics(4.0, 0.0, 0.0, 0.0);

    assertEquals(
        0.0,
        sol.deltaAzimuthDeg(),
        ANGLE_TOLERANCE_DEG,
        "[Calculated Approach] Azimuth correction should be near zero for a stationary robot aimed at target");
  }

  @Test
  @DisplayName("[Calculated Approach] Moving robot: launch speed differs from stationary shot")
  public void testMovingRobot_speedDiffersFromStationary() {
    BallisticSolution stationary = ShootingUtil.computeBallistics(4.0, 0.0, 0.0, 0.0);
    // Robot moving toward the target at 3 m/s
    BallisticSolution moving = ShootingUtil.computeBallistics(4.0, 0.0, 3.0, 0.0);

    // Shooter output speed must be lower when moving toward target (Galilean subtraction)
    assertTrue(
        moving.launchSpeed() < stationary.launchSpeed(),
        "[Calculated Approach] Shooter output should be lower when robot moves toward the target");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Moving robot: azimuth correction applied for lateral velocity")
  public void testMovingRobot_lateralAzimuthCorrection() {
    // Robot moving laterally (perpendicular to target direction)
    BallisticSolution sol = ShootingUtil.computeBallistics(4.0, 0.0, 0.0, 3.0);

    // Shooter must aim off-axis to compensate
    assertNotEquals(
        0.0,
        sol.deltaAzimuthDeg(),
        ANGLE_TOLERANCE_DEG,
        "[Calculated Approach] Lateral robot velocity must produce a nonzero azimuth correction");
  }

  @Test
  @DisplayName("[Calculated Approach] Longer range produces higher required launch speed")
  public void testLaunchSpeedScalesWithDistance() {
    BallisticSolution near = ShootingUtil.computeBallistics(2.0, 0.0, 0.0, 0.0);
    BallisticSolution far = ShootingUtil.computeBallistics(6.0, 0.0, 0.0, 0.0);

    assertTrue(
        far.launchSpeed() > near.launchSpeed(), "Farther targets require higher launch speed");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Non-zero azimuth target: solution points shooter toward target")
  public void testNonZeroAzimuth_solutionDirectionCorrect() {
    double azimuthDeg = 45.0;
    BallisticSolution sol = ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0);

    // deltaAzimuthDeg is the correction relative to azimuth; for stationary robot it should be ~0
    assertEquals(
        0.0,
        sol.deltaAzimuthDeg(),
        ANGLE_TOLERANCE_DEG,
        "[Calculated Approach] No azimuth correction needed for stationary robot regardless of target direction");
    assertTrue(
        sol.launchSpeed() > 0,
        "[Calculated Approach] Launch speed must be positive for angled target");
  }

  // ---------------------------------------------------------------------------
  // computeBallistics (acceleration overload)
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: zero acceleration matches no-acceleration overload")
  public void testAccelOverload_zeroAccelMatchesBaseOverload() {
    double D = 4.0;
    BallisticSolution base = ShootingUtil.computeBallistics(D, 0.0, 2.0, 0.0);
    BallisticSolution accel = ShootingUtil.computeBallistics(D, 0.0, 2.0, 0.0, 0.0, 0.0);
    System.out.println("speeds " + base.launchSpeed() + " " + accel.launchSpeed());
    System.out.println("hood angles " + base.hoodAngleDeg() + " " + accel.hoodAngleDeg());
    System.out.println("azimuths " + base.deltaAzimuthDeg() + " " + accel.deltaAzimuthDeg());

    assertEquals(
        base.launchSpeed(),
        accel.launchSpeed(),
        SPEED_TOLERANCE_MPS,
        "[Calculated Approach] Zero-acceleration overload should match base overload launch speed");
    assertEquals(
        base.hoodAngleDeg(),
        accel.hoodAngleDeg(),
        ANGLE_TOLERANCE_DEG,
        "[Calculated Approach] Zero-acceleration overload should match base overload hood angle");
    assertEquals(
        base.deltaAzimuthDeg(),
        accel.deltaAzimuthDeg(),
        ANGLE_TOLERANCE_DEG,
        "[Calculated Approach] Zero-acceleration overload should match base overload azimuth delta");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: nonzero X acceleration changes the solution")
  public void testAccelOverload_nonzeroXaccelChangesSolution() {
    double D = 4.0;
    BallisticSolution noAccel = ShootingUtil.computeBallistics(D, 0.0, 0.0, 0.0, 0.0, 0.0);
    BallisticSolution withAccel = ShootingUtil.computeBallistics(D, 0.0, 0.0, 0.0, 4.0, 0.0);

    // At max robot acceleration the solution must shift meaningfully
    assertNotEquals(
        noAccel.launchSpeed(),
        withAccel.launchSpeed(),
        SPEED_TOLERANCE_MPS,
        "[Calculated Approach] Nonzero acceleration should produce a different launch speed");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: nonzero Y acceleration changes the solution")
  public void testAccelOverload_nonzeroYaccelChangesSolution() {
    double D = 4.0;
    BallisticSolution noAccel = ShootingUtil.computeBallistics(D, 0.0, 0.0, 0.0, 0.0, 0.0);
    BallisticSolution withAccel = ShootingUtil.computeBallistics(D, 0.0, 0.0, 0.0, 0.0, 4.0);

    // At max robot acceleration the solution must shift meaningfully
    assertNotEquals(
        noAccel.deltaAzimuthDeg(),
        withAccel.deltaAzimuthDeg(),
        SPEED_TOLERANCE_MPS,
        "[Calculated Approach] Nonzero acceleration should produce a different launch speed");
  }

  @Test
  @DisplayName("[Calculated Approach] Acceleration overload: solution is physically plausible")
  public void testAccelOverload_plausibility() {
    BallisticSolution sol = ShootingUtil.computeBallistics(4.0, 0.0, 2.0, 1.0, 3.0, 1.5);

    assertTrue(
        sol.launchSpeed() > 0,
        "[Calculated Approach] Launch speed must be positive under acceleration");
    assertTrue(
        sol.hoodAngleDeg() > 0 && sol.hoodAngleDeg() < 90,
        "[Calculated Approach] Hood angle must remain in physical range under acceleration");
  }

  // ---------------------------------------------------------------------------
  // predictLandingPose
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] predictLandingPose: zero initial velocity drops straight down")
  public void testPredictLanding_zeroPlanarVelocity() {
    double[] landing = ShootingUtil.predictLandingPose(0, 0, 0, 0, 5.0);

    // With no horizontal velocity the ball should land very close to start XY
    assertEquals(
        0.0, landing[0], 0.1, "[Calculated Approach] X should not drift with zero planar velocity");
    assertEquals(
        0.0, landing[1], 0.1, "[Calculated Approach] Y should not drift with zero planar velocity");
  }

  @Test
  @DisplayName(
      "[Calculated Approach] predictLandingPose: xStart/yStart offset is preserved in output")
  public void testPredictLanding_startOffsetPreserved() {
    double[] fromOrigin = ShootingUtil.predictLandingPose(0, 0, 5.0, 0.0, 3.0);
    double[] fromOffset = ShootingUtil.predictLandingPose(2.0, 1.0, 5.0, 0.0, 3.0);

    // The X offset should shift the landing by exactly the start offset
    assertEquals(
        fromOrigin[0] + 2.0,
        fromOffset[0],
        POSITION_TOLERANCE_M,
        "[Calculated Approach] X landing should shift by xStart offset");
    assertEquals(
        fromOrigin[1] + 1.0,
        fromOffset[1],
        POSITION_TOLERANCE_M,
        "[Calculated Approach] Y landing should shift by yStart offset");
  }

  // ---------------------------------------------------------------------------
  // computeMotorAdjustment
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] computeMotorAdjustment: returns positive RPM for positive launch speed")
  public void testMotorAdjustment_positive() {
    double omega = ShootingUtil.computeMotorAdjustment(10.0);
    assertTrue(
        omega > 0,
        "[Calculated Approach] Motor angular velocity must be positive for positive launch speed");
  }

  @Test
  @DisplayName("[Calculated Approach] computeMotorAdjustment: scales linearly with launch speed")
  public void testMotorAdjustment_linearScaling() {
    double omega1 = ShootingUtil.computeMotorAdjustment(10.0);
    double omega2 = ShootingUtil.computeMotorAdjustment(20.0);

    assertEquals(
        2.0,
        omega2 / omega1,
        0.01,
        "[Calculated Approach] Motor adjustment should scale linearly with launch speed");
  }
}
