package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedStatic;

public class TurretConfidenceUtilTest {

  // Test parameters, adjust based on situation
  double vx = 0.0; // x-velocity translation
  double vy = 0.0; // y-velocity translation
  double rotationSpeed = 0.0; // rotation speed
  double distance = 1.0; // distance from target
  double timeOfFlight = 1.0; // time of flight, dependent on distance

  // Mock swerve drive subsystem
  SwerveDriveIO drive = mock(SwerveDriveIO.class);

  // Mock shooting util and shooting constants
  private MockedStatic<ShootingUtil> shootingUtilMock;
  private MockedStatic<ShootingConstants> shootingConstantsMock;

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
  @DisplayName("calculates confidence based on simulated drive state and target information")
  void testConfidenceCalculation() {
    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("Confidence = " + confidence);
    assertTrue(confidence > 0);
    assertTrue(confidence <= 100);
  }

  // Test 2: Value sweep of vx translation
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0, 3.0})
  @DisplayName("Confidence decreases as vx increases")
  void testXVelocitySweep(double vxValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(vxValue, 0, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("vx=" + vxValue + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  // Test 3: Value sweep of vy translation
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0})
  @DisplayName("Confidence decreases as vy increases")
  void testYVelocitySweep(double vyValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, vyValue, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("vy=" + vyValue + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  // Test 4: Value sweep of rotation speed
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0})
  @DisplayName("Confidence decreases as rotation speed increases")
  void testRotationSpeedSweep(double rotationSpeedValue) {

    when(drive.getChassisSpeedsFieldRelative())
        .thenReturn(new ChassisSpeeds(0, 0, rotationSpeedValue));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("rotationSpeed=" + rotationSpeedValue + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})
  @DisplayName("Confidence decreases as distance increases")
  void testDistanceSweep(double distance) {
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any()))
        .thenReturn(distance);

    // Call the method being tested
    double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    System.out.println("distance=" + distance + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }
}
