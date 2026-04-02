package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import java.util.stream.Stream;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.MockedStatic;

public class TurretConfidenceUtilTest {

  // Mock swerve drive subsystem
  SwerveDriveIO drive = mock(SwerveDriveIO.class);

  // Mock shooting util and shooting constants
  private MockedStatic<ShootingUtil> shootingUtilMock;
  private MockedStatic<ShootingConstants> shootingConstantsMock;

  @BeforeEach
  void setup() {
    // Test parameters for mocks - change these as needed to test different scenarios
    double vx = 2.0; // x-translation speed (m/s)
    double vy = 0.0; // y-translation speed (m/s)
    double rotationSpeed = 0.0; // angular rotation speed (rad/s)
    double distance = 1.0; // distance from hub meters
    double timeOfFlight = 1.0; // seconds, based on distance

    // --- Drive pose and speed mocks ---
    Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationSpeed);

    when(drive.getPose()).thenReturn(pose);
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(speeds);

    // --- Static mocks ---
    shootingUtilMock = mockStatic(ShootingUtil.class);
    shootingConstantsMock = mockStatic(ShootingConstants.class);

    Translation2d target = new Translation2d(distance, 0);
    Pose2d targetPose = new Pose2d(target, new Rotation2d());

    shootingUtilMock.when(() -> ShootingUtil.getShootingTarget(any(), anyBoolean())).thenReturn(targetPose);

    // Set up default distance and time of flight mocks
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any(), anyBoolean()))
        .thenReturn(distance);
    shootingConstantsMock
        .when(() -> ShootingConstants.getTimeOfFlight(distance))
        .thenReturn(timeOfFlight);
  }

  @AfterEach
  void cleanup() {
    shootingUtilMock.close();
    shootingConstantsMock.close();
    clearInvocations(drive);
  }

  @ParameterizedTest
  @MethodSource("provideVxAndMinConfidence")
  @DisplayName("Confidence meets minimum threshold for given vx")
  void testXVelocitySweep(double vx, double expectedMinConfidence) {
    // Setup
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(vx, 0, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.printf(
        "vx=%.1f confidence=%.2f%% (min expected: %.2f%%)", vx, confidence, expectedMinConfidence);

    assertTrue(
        confidence > expectedMinConfidence,
        String.format(
            "For vx=%.1f m/s, confidence %.2f%% should be above minimum threshold %.2f%%",
            vx, confidence, expectedMinConfidence));
  }

  private static Stream<Arguments> provideVxAndMinConfidence() {
    return Stream.of(
        Arguments.of(0.0, 90.0),
        Arguments.of(0.5, 80.0),
        Arguments.of(1.0, 70.0),
        Arguments.of(2.0, 50.0),
        Arguments.of(3.0, 30.0));
  }

  @ParameterizedTest
  @MethodSource("provideVyAndMinConfidence")
  @DisplayName("Confidence meets minimum threshold for given vy")
  void testYVelocitySweep(double vy, double expectedMinConfidence) {
    // Setup
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, vy, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.printf(
        "vy=%.1f confidence=%.2f%% (min expected: %.2f%%)", vy, confidence, expectedMinConfidence);

    assertTrue(
        confidence > expectedMinConfidence,
        String.format(
            "For vy=%.1f m/s, confidence %.2f%% should be above minimum threshold %.2f%%",
            vy, confidence, expectedMinConfidence));
  }

  private static Stream<Arguments> provideVyAndMinConfidence() {
    return Stream.of(
        Arguments.of(0.0, 90.0),
        Arguments.of(0.5, 80.0),
        Arguments.of(1.0, 70.0),
        Arguments.of(2.0, 50.0));
  }

  @ParameterizedTest
  @MethodSource("provideRotationAndMinConfidence")
  @DisplayName("Confidence meets minimum threshold for given rotation speed")
  void testRotationSpeedSweep(double rotationSpeed, double expectedMinConfidence) {
    // Setup
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, 0, rotationSpeed));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.printf(
        "rotationSpeed=%.1f confidence=%.2f%% (min expected: %.2f%%)",
        rotationSpeed, confidence, expectedMinConfidence);

    assertTrue(
        confidence > expectedMinConfidence,
        String.format(
            "For rotationSpeed=%.1f rad/s, confidence %.2f%% should be above minimum threshold %.2f%%",
            rotationSpeed, confidence, expectedMinConfidence));
  }

  private static Stream<Arguments> provideRotationAndMinConfidence() {
    return Stream.of(
        Arguments.of(0.0, 90.0),
        Arguments.of(0.5, 85.0),
        Arguments.of(1.0, 75.0),
        Arguments.of(2.0, 50.0));
  }

  @ParameterizedTest
  @MethodSource("provideDistanceAndMinConfidence")
  @DisplayName("Confidence meets minimum threshold for given distance")
  void testDistanceSweep(double distance, double expectedMinConfidence) {
    // Setup - stationary robot for distance test
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, 0, 0));
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any(), anyBoolean()))
        .thenReturn(distance);
    shootingConstantsMock
        .when(() -> ShootingConstants.getTimeOfFlight(distance))
        .thenReturn(distance * 0.2); // Simple TOF calculation based on distance

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.printf(
        "distance=%.1f confidence=%.2f%% (min expected: %.2f%%)",
        distance, confidence, expectedMinConfidence);

    assertTrue(
        confidence > expectedMinConfidence,
        String.format(
            "For distance=%.1f m, confidence %.2f%% should be above minimum threshold %.2f%%",
            distance, confidence, expectedMinConfidence));
  }

  private static Stream<Arguments> provideDistanceAndMinConfidence() {
    return Stream.of(
        Arguments.of(1.0, 95.0),
        Arguments.of(2.0, 90.0),
        Arguments.of(3.0, 85.0),
        Arguments.of(4.0, 80.0),
        Arguments.of(5.0, 75.0),
        Arguments.of(6.0, 65.0),
        Arguments.of(7.0, 55.0),
        Arguments.of(8.0, 45.0));
  }

  @ParameterizedTest
  @MethodSource("provideCombinedParamsAndMinConfidence")
  @DisplayName("Confidence meets minimum threshold for combined conditions")
  void testCombinedParameters(
      double vx, double vy, double rotation, double distance, double expectedMinConfidence) {
    // Setup
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(vx, vy, rotation));
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any(), anyBoolean()))
        .thenReturn(distance);
    shootingConstantsMock
        .when(() -> ShootingConstants.getTimeOfFlight(distance))
        .thenReturn(distance * 0.2); // Simple TOF calculation

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.printf(
        "vx=%.1f vy=%.1f rot=%.1f dist=%.1f -> confidence=%.2f%% (min exp=%.2f%%)",
        vx, vy, rotation, distance, confidence, expectedMinConfidence);

    assertTrue(
        confidence > expectedMinConfidence,
        String.format(
            "For combined conditions, confidence %.2f%% should be above minimum threshold %.2f%%",
            confidence, expectedMinConfidence));
  }

  private static Stream<Arguments> provideCombinedParamsAndMinConfidence() {
    return Stream.of(
        // Ideal conditions
        Arguments.of(0.0, 0.0, 0.0, 2.0, 90.0),
        // Moving slowly, close target
        Arguments.of(0.5, 0.5, 0.2, 3.0, 85.0),
        // Moving faster, medium target
        Arguments.of(1.0, 0.5, 0.5, 4.0, 75.0),
        // Moving fast, far target
        Arguments.of(2.0, 1.0, 1.0, 6.0, 60.0),
        // Near maximum limits
        Arguments.of(3.0, 2.0, 2.0, 7.0, 45.0));
  }
}
