package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.util.ShootingUtil.BallisticSolution;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;

public class ShootingUtilTest {

  // Tolerance in meters for landing position assertions.
  private static final double POSITION_TOLERANCE_M = 0.05;

  // Tolerance in m/s for launch speed assertions.
  private static final double SPEED_TOLERANCE_MPS = 0.01;

  // Tolerance in degrees for angle assertions.
  private static final double ANGLE_TOLERANCE_DEG = 0.01;

  @BeforeEach
  public void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  // ---------------------------------------------------------------------------
  // computeBallistics — stationary robot
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName("[Calculated Approach] Stationary robot: solution lands within position tolerance")
  public void testStationaryShot_landingTolerance() {
    List<Executable> assertions = new ArrayList<>();

    for (double d = 1.0; d <= 5.0; d += 0.5) {
      for (int az = 0; az < 360; az += 5) {
        final double dist = d;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution sol = ShootingUtil.computeBallistics(dist, azimuthDeg, 0.0, 0.0);

              double azRad = Math.toRadians(azimuthDeg);
              double absoluteAzimuthRad = Math.toRadians(sol.deltaAzimuthDeg()) + azRad;
              double hoodRad = Math.toRadians(sol.hoodAngleDeg());

              double vxLaunch =
                  sol.launchSpeed() * Math.cos(hoodRad) * Math.cos(absoluteAzimuthRad);
              double vyLaunch =
                  sol.launchSpeed() * Math.cos(hoodRad) * Math.sin(absoluteAzimuthRad);
              double vzLaunch = sol.launchSpeed() * Math.sin(hoodRad);

              double[] landing =
                  ShootingUtil.predictLandingPose(0, 0, vxLaunch, vyLaunch, vzLaunch);

              assertAll(
                  "d=" + dist + " az=" + azimuthDeg,
                  () ->
                      assertEquals(
                          dist * Math.cos(azRad),
                          landing[0],
                          POSITION_TOLERANCE_M,
                          "X landing out of tolerance"),
                  () ->
                      assertEquals(
                          dist * Math.sin(azRad),
                          landing[1],
                          POSITION_TOLERANCE_M,
                          "Y landing out of tolerance"));
            });
      }
    }

    assertAll("[Calculated Approach] landing tolerance", assertions.stream());
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Stationary robot: solution is physically plausible across distances and azimuths")
  public void testStationaryShot_plausibility() {
    List<Executable> assertions = new ArrayList<>();

    for (double d = 1.0; d <= 5.0; d += 0.5) {
      for (int az = 0; az < 360; az += 5) {
        final double dist = d;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution sol = ShootingUtil.computeBallistics(dist, azimuthDeg, 0.0, 0.0);
              assertAll(
                  "d=" + dist + " az=" + azimuthDeg,
                  () -> assertTrue(sol.launchSpeed() > 0, "Launch speed must be positive"),
                  () ->
                      assertTrue(
                          sol.hoodAngleDeg() > 0 && sol.hoodAngleDeg() < 90,
                          "Hood angle must be between 0 and 90 degrees"));
            });
      }
    }

    assertAll("[Calculated Approach] stationary plausibility", assertions.stream());
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Stationary robot: azimuth correction is zero when aiming directly at target")
  public void testStationaryShot_noAzimuthCorrection() {
    List<Executable> assertions = new ArrayList<>();

    for (double d = 1.0; d <= 5.0; d += 0.5) {
      for (int az = 0; az < 360; az += 5) {
        final double dist = d;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution sol = ShootingUtil.computeBallistics(dist, azimuthDeg, 0.0, 0.0);

              // Normalize to [-180, 180] to handle atan2 wrapping
              double normalized = sol.deltaAzimuthDeg() % 360.0;
              if (normalized > 180.0) normalized -= 360.0;
              if (normalized < -180.0) normalized += 360.0;
              final double norm = normalized;

              assertEquals(
                  0.0,
                  norm,
                  ANGLE_TOLERANCE_DEG,
                  "Azimuth correction non-zero at d=" + dist + " az=" + azimuthDeg);
            });
      }
    }

    assertAll("[Calculated Approach] no azimuth correction stationary", assertions.stream());
  }

  @Test
  @DisplayName("[Calculated Approach] Longer range produces higher required launch speed")
  public void testLaunchSpeedScalesWithDistance() {
    List<Executable> assertions = new ArrayList<>();

    for (int az = 0; az < 360; az += 5) {
      final double azimuthDeg = az;

      assertions.add(
          () -> {
            BallisticSolution near = ShootingUtil.computeBallistics(1.0, azimuthDeg, 0.0, 0.0);
            BallisticSolution mid = ShootingUtil.computeBallistics(3.0, azimuthDeg, 0.0, 0.0);
            BallisticSolution far = ShootingUtil.computeBallistics(5.0, azimuthDeg, 0.0, 0.0);

            assertAll(
                "az=" + azimuthDeg,
                () ->
                    assertTrue(
                        mid.launchSpeed() > near.launchSpeed(),
                        "Mid-range should require more speed than near"),
                () ->
                    assertTrue(
                        far.launchSpeed() > mid.launchSpeed(),
                        "Far should require more speed than mid-range"));
          });
    }

    assertAll("[Calculated Approach] speed scales with distance", assertions.stream());
  }

  // ---------------------------------------------------------------------------
  // computeBallistics — moving robot
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] Moving robot: approaching target reduces required shooter speed")
  public void testMovingRobot_speedDiffersFromStationary() {
    List<Executable> assertions = new ArrayList<>();

    for (double v = 0.5; v <= 4.0; v += 0.5) {
      for (int az = 0; az < 360; az += 15) {
        final double vel = v;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution stationary =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0);
              double vxField = vel * Math.cos(Math.toRadians(azimuthDeg));
              double vyField = vel * Math.sin(Math.toRadians(azimuthDeg));
              BallisticSolution moving =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, vxField, vyField);

              assertTrue(
                  moving.launchSpeed() < stationary.launchSpeed(),
                  "Shooter output should be lower when moving toward target at v="
                      + vel
                      + " az="
                      + azimuthDeg);
            });
      }
    }

    assertAll("[Calculated Approach] moving robot speed reduction", assertions.stream());
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Moving robot: azimuth correction applied for lateral velocity")
  public void testMovingRobot_lateralAzimuthCorrection() {
    List<Executable> assertions = new ArrayList<>();

    for (double v = 0.5; v <= 4.0; v += 0.5) {
      for (int az = 0; az < 360; az += 15) {
        final double vel = v;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              double azRad = Math.toRadians(azimuthDeg);
              double vxLateral = vel * -Math.sin(azRad);
              double vyLateral = vel * Math.cos(azRad);

              BallisticSolution sol =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, vxLateral, vyLateral);

              assertNotEquals(
                  0.0,
                  sol.deltaAzimuthDeg(),
                  ANGLE_TOLERANCE_DEG,
                  "Lateral velocity must produce nonzero azimuth correction at v="
                      + vel
                      + " az="
                      + azimuthDeg);
            });
      }
    }

    assertAll("[Calculated Approach] lateral velocity azimuth correction", assertions.stream());
  }

  // ---------------------------------------------------------------------------
  // computeBallistics — acceleration overload
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: zero acceleration matches no-acceleration overload")
  public void testAccelOverload_zeroAccelMatchesBaseOverload() {
    List<Executable> assertions = new ArrayList<>();

    for (double d = 1.0; d <= 5.0; d += 1.0) {
      for (int az = 0; az < 360; az += 15) {
        final double dist = d;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution base = ShootingUtil.computeBallistics(dist, azimuthDeg, 0.0, 0.0);
              BallisticSolution accel =
                  ShootingUtil.computeBallistics(dist, azimuthDeg, 0.0, 0.0, 0.0, 0.0);

              assertAll(
                  "d=" + dist + " az=" + azimuthDeg,
                  () ->
                      assertEquals(
                          base.launchSpeed(),
                          accel.launchSpeed(),
                          SPEED_TOLERANCE_MPS,
                          "Launch speed mismatch"),
                  () ->
                      assertEquals(
                          base.hoodAngleDeg(),
                          accel.hoodAngleDeg(),
                          ANGLE_TOLERANCE_DEG,
                          "Hood angle mismatch"),
                  () ->
                      assertEquals(
                          base.deltaAzimuthDeg(),
                          accel.deltaAzimuthDeg(),
                          ANGLE_TOLERANCE_DEG,
                          "Azimuth delta mismatch"));
            });
      }
    }

    assertAll("[Calculated Approach] zero accel equivalence", assertions.stream());
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: nonzero X acceleration changes the solution")
  public void testAccelOverload_nonzeroXaccelChangesSolution() {
    List<Executable> assertions = new ArrayList<>();

    for (double aX = 1.0; aX <= 4.0; aX += 1.0) {
      for (int az = 0; az < 360; az += 5) {
        // Skip angles where X acceleration is perpendicular to target —
        // at these angles it affects azimuth, not launch speed magnitude
        if ((80 <= az && az <= 100) || (260 <= az && az <= 280)) continue;

        final double accelX = aX;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution noAccel =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0, 0.0, 0.0);
              BallisticSolution withAccel =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0, accelX, 0.0);

              assertNotEquals(
                  noAccel.launchSpeed(),
                  withAccel.launchSpeed(),
                  SPEED_TOLERANCE_MPS,
                  "X acceleration should change launch speed at aX="
                      + accelX
                      + " az="
                      + azimuthDeg);
            });
      }
    }

    assertAll("[Calculated Approach] X acceleration changes solution", assertions.stream());
  }

  @Test
  @DisplayName(
      "[Calculated Approach] Acceleration overload: nonzero Y acceleration changes the azimuth")
  public void testAccelOverload_nonzeroYaccelChangesSolution() {
    List<Executable> assertions = new ArrayList<>();

    for (double aY = 1.0; aY <= 4.0; aY += 1.0) {
      for (int az = 0; az < 360; az += 5) {
        // Skip angles where Y acceleration is axial to target —
        // at 0, 90, 180, 270 it changes speed rather than azimuth
        if (az == 0 || az == 90 || az == 180 || az == 270) continue;

        final double accelY = aY;
        final double azimuthDeg = az;

        assertions.add(
            () -> {
              BallisticSolution noAccel =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0, 0.0, 0.0);
              BallisticSolution withAccel =
                  ShootingUtil.computeBallistics(4.0, azimuthDeg, 0.0, 0.0, 0.0, accelY);

              assertNotEquals(
                  noAccel.deltaAzimuthDeg(),
                  withAccel.deltaAzimuthDeg(),
                  ANGLE_TOLERANCE_DEG,
                  "Y acceleration should shift azimuth at aY=" + accelY + " az=" + azimuthDeg);
            });
      }
    }

    assertAll("[Calculated Approach] Y acceleration changes azimuth", assertions.stream());
  }

  @Test
  @DisplayName("[Calculated Approach] Acceleration overload: solution is physically plausible")
  public void testAccelOverload_plausibility() {
    List<Executable> assertions = new ArrayList<>();

    for (double d = 1.0; d <= 5.0; d += 1.0) {
      for (int az = 0; az < 360; az += 45) {
        for (double a = 0.0; a <= 4.0; a += 2.0) {
          final double dist = d;
          final double azimuthDeg = az;
          final double accel = a;

          assertions.add(
              () -> {
                BallisticSolution sol =
                    ShootingUtil.computeBallistics(dist, azimuthDeg, 2.0, 1.0, accel, accel);

                assertAll(
                    "d=" + dist + " az=" + azimuthDeg + " a=" + accel,
                    () -> assertTrue(sol.launchSpeed() > 0, "Launch speed must be positive"),
                    () ->
                        assertTrue(
                            sol.hoodAngleDeg() > 0 && sol.hoodAngleDeg() < 90,
                            "Hood angle must remain in physical range"));
              });
        }
      }
    }

    assertAll("[Calculated Approach] acceleration plausibility", assertions.stream());
  }

  // ---------------------------------------------------------------------------
  // predictLandingPose
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] predictLandingPose: zero initial planar velocity drops straight down")
  public void testPredictLanding_zeroPlanarVelocity() {
    double[] landing = ShootingUtil.predictLandingPose(0, 0, 0, 0, 5.0);

    assertAll(
        () -> assertEquals(0.0, landing[0], 0.1, "X should not drift with zero planar velocity"),
        () -> assertEquals(0.0, landing[1], 0.1, "Y should not drift with zero planar velocity"));
  }

  @Test
  @DisplayName(
      "[Calculated Approach] predictLandingPose: xStart/yStart offset is preserved in output")
  public void testPredictLanding_startOffsetPreserved() {
    List<Executable> assertions = new ArrayList<>();

    for (double x = -3.0; x <= 3.0; x += 1.0) {
      for (double y = -3.0; y <= 3.0; y += 1.0) {
        final double xStart = x;
        final double yStart = y;

        assertions.add(
            () -> {
              double[] fromOrigin = ShootingUtil.predictLandingPose(0, 0, 5.0, 0.0, 3.0);
              double[] fromOffset = ShootingUtil.predictLandingPose(xStart, yStart, 5.0, 0.0, 3.0);

              assertAll(
                  "xStart=" + xStart + " yStart=" + yStart,
                  () ->
                      assertEquals(
                          fromOrigin[0] + xStart,
                          fromOffset[0],
                          POSITION_TOLERANCE_M,
                          "X landing should shift by xStart"),
                  () ->
                      assertEquals(
                          fromOrigin[1] + yStart,
                          fromOffset[1],
                          POSITION_TOLERANCE_M,
                          "Y landing should shift by yStart"));
            });
      }
    }

    assertAll("[Calculated Approach] start offset preserved", assertions.stream());
  }

  // ---------------------------------------------------------------------------
  // computeMotorAdjustment
  // ---------------------------------------------------------------------------

  @Test
  @DisplayName(
      "[Calculated Approach] computeMotorAdjustment: returns positive RPM for positive launch speed")
  public void testMotorAdjustment_positive() {
    List<Executable> assertions = new ArrayList<>();

    for (double s = 1.0; s <= 20.0; s += 1.0) {
      final double speed = s;
      assertions.add(
          () ->
              assertTrue(
                  ShootingUtil.computeMotorAdjustment(speed) > 0,
                  "Motor adjustment must be positive for launchSpeed=" + speed));
    }

    assertAll("[Calculated Approach] motor adjustment positive", assertions.stream());
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
