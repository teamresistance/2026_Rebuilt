package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedStatic;

@TestMethodOrder(OrderAnnotation.class)
public class TurretConfidenceUtilTest {

  // Test parameters, change as needed to test different scenarios
  double vx = 1.0; // x-translation speed (m/s)
  double vy = 0.0; // y-translation speed (m/s)
  double rotationSpeed = 0.0; // angular rotation speed (rad/s)
  double distance = 5.0; // distance from hub meters
  double timeOfFlight = 1.0; // seconds, based on distance

  // Mock swerve drive subsystem
  SwerveDriveIO drive = mock(SwerveDriveIO.class);

  // Mock shooting util and shooting constants
  private MockedStatic<ShootingUtil> shootingUtilMock;
  private MockedStatic<ShootingConstants> shootingConstantsMock;

  // Store previous confidence values for comparison
  private static double previousVxConfidence = 0.0;
  private static double previousVyConfidence = 0.0;
  private static double previousRotationConfidence = 0.0;
  private static double previousDistanceConfidence = 0.0;

  @BeforeEach
  void setup() {

    // --- Drive pose and speed mocks ---
    Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationSpeed);

    when(drive.getPose()).thenReturn(pose);
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(speeds);

    // --- Static mocks ---
    shootingUtilMock = mockStatic(ShootingUtil.class);
    shootingConstantsMock = mockStatic(ShootingConstants.class);

    Translation2d target = new Translation2d(6, 0);
    Pose2d targetPose = new Pose2d(target, new Rotation2d());

    shootingUtilMock.when(() -> ShootingUtil.getShootingTarget(any())).thenReturn(targetPose);

    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any()))
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

  // Test 1: Calculates confidence
  @Test
  @Order(0)
  @DisplayName("calculates confidence based on simulated drive state and target information")
  void testConfidenceCalculation() {
    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("Confidence = " + confidence);
    assertTrue(
        confidence > 75,
        "Confidence should be more that 75% to be acceptable"); // only pass test if confidence is
    // acceptable (above 75%)
    assertTrue(confidence <= 100, "Confidence should not exceed 100%");
  }

  // Test 2: Value sweep of vx translation
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0, 3.0})
  @Order(1)
  @DisplayName("Confidence decreases as vx increases")
  void testXVelocitySweep(double vxValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(vxValue, 0, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("vx=" + vxValue + " confidence=" + confidence);

    // Compare with previous value
    if (previousVxConfidence != 0.0) {
      assertTrue(
          confidence <= previousVxConfidence + 0.01,
          String.format("vx increased, confidence should decrease"));
    }

    // Store for next comparison
    previousVxConfidence = confidence;

    // Basic sanity check
    assertTrue(confidence >= 0 && confidence <= 100);
  }

  // Test 3: Value sweep of vy translation
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0})
  @Order(2)
  @DisplayName("Confidence decreases as vy increases")
  void testYVelocitySweep(double vyValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, vyValue, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("vy=" + vyValue + " confidence=" + confidence);

    // Compare with previous value
    if (previousVyConfidence != 0.0) {
      assertTrue(
          confidence <= previousVyConfidence + 0.01,
          String.format("vy increased, confidence should decrease"));
    }

    // Store for next comparison
    previousVyConfidence = confidence;

    // Basic sanity check
    assertTrue(confidence >= 0 && confidence <= 100);
  }

  // Test 4: Value sweep of rotation speed
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0})
  @Order(3)
  @DisplayName("Confidence decreases as rotation speed increases")
  void testRotationSpeedSweep(double rotationSpeedValue) {

    when(drive.getChassisSpeedsFieldRelative())
        .thenReturn(new ChassisSpeeds(0, 0, rotationSpeedValue));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("rotationSpeed=" + rotationSpeedValue + " confidence=" + confidence);

    // Compare with previous value (skip first iteration)
    if (previousRotationConfidence != 0.0) {
      assertTrue(
          confidence <= previousRotationConfidence + 0.01,
          String.format("rotation speed increased, confidence should decrease"));
    }

    // Store for next comparison
    previousRotationConfidence = confidence;

    // Basic sanity check
    assertTrue(confidence >= 0 && confidence <= 100);
  }

  // Test 5: Value sweep of distance
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})
  @Order(4)
  @DisplayName("Confidence decreases as distance increases")
  void testDistanceSweep(double distance) {
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any()))
        .thenReturn(distance);

    // Update time of flight based on distance
    shootingConstantsMock
        .when(() -> ShootingConstants.getTimeOfFlight(distance))
        .thenReturn(distance * 0.2); // Simple TOF calculation

    // Call the method being tested
    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("distance=" + distance + " confidence=" + confidence);

    // Compare with previous value (skip first iteration)
    if (previousDistanceConfidence != 0.0) {
      assertTrue(
          confidence <= previousDistanceConfidence + 0.01,
          String.format("distance increased, confidence should decrease"));
    }

    // Store for next comparison
    previousDistanceConfidence = confidence;

    // Basic sanity check
    assertTrue(confidence >= 0 && confidence <= 100);
  }
}
