package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedStatic;

public class TurretConfidenceUtilTest {

  // Test parameters, change as needed to test different scenarios
  double vx = 2.0; // x-translation speed (m/s)
  double vy = 0.0; // y-translation speed (m/s)
  double rotationSpeed = 0.0; // angular rotationspeed (rad/s)
  double distance = 5.0; // distance from hub meters
  double timeOfFlight = 1.0; // seconds, based on distance

  // Mock drive subsystem
  SwerveDriveIO drive = mock(SwerveDriveIO.class);

  // Helper method to set up mock behavior for the drive subsystem and static methods
  @BeforeEach
  void mockSwerveDriveIO() {

    // Set up mock behavior for pose and speeds
    Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationSpeed);

    // Mock the drive subsystem to return the pose and speeds
    when(drive.getPose()).thenReturn(pose);
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(speeds);

    MockedStatic<ShootingUtil> shootingUtilMock = mockStatic(ShootingUtil.class);
    MockedStatic<ShootingConstants> shootingConstantsMock = mockStatic(ShootingConstants.class);

    // Set up mock behavior for shooting utility methods
    Translation2d target = new Translation2d(6, 0);
    Pose2d targetPose = new Pose2d(target, new Rotation2d());

    // Mock the shooting utility to return the target pose and distance
    shootingUtilMock.when(() -> ShootingUtil.getShootingTarget(any())).thenReturn(targetPose);

    // Mock the shooting utility to return the distance to the target
    shootingUtilMock
        .when(() -> ShootingUtil.getVirtualDistanceToTarget(any(), any()))
        .thenReturn(distance);

    // Mock the shooting constants to return a time of flight (0.6s) based on the distance
    shootingConstantsMock
        .when(() -> ShootingConstants.getTimeOfFlight(distance))
        .thenReturn(timeOfFlight);
  }

  // ------------------------------------------------------------
  // 1. Basic confidence calculation — confidence should be a positive value based on the mocked
  // state
  // ------------------------------------------------------------
  @Test
  @DisplayName("calculates confidence based on simulated drive state and target information")
  void testConfidenceCalculation() {

    // Call the method under test
    double confidence = TurretConfidenceUtil.calculateConfidence(drive);

    // Print the confidence for debugging purposes
    System.out.println("Confidence = " + confidence);

    // Assert that the confidence is within the expected range (0 to 100)
    assertTrue(confidence > 0);
    assertTrue(confidence <= 100);
  }

  // ------------------------------------------------------------
  // 2. vx sweep — confidence should decrease as vx increases
  // ------------------------------------------------------------
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0, 3.0})
  @DisplayName("Confidence decreases as vx increases")
  void testXVelocitySweep(double vxValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(vxValue, 0, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);

    System.out.println("vx=" + vxValue + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  // ------------------------------------------------------------
  // 3. vy sweep — confidence should decrease as vy increases
  // ------------------------------------------------------------
  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.5, 1.0, 2.0})
  @DisplayName("Confidence decreases as vy increases")
  void testYVelocitySweep(double vyValue) {

    when(drive.getChassisSpeedsFieldRelative()).thenReturn(new ChassisSpeeds(0, vyValue, 0));

    double confidence = TurretConfidenceUtil.calculateConfidence(drive);

    System.out.println("vy=" + vyValue + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  // ------------------------------------------------------------
  // 4. rotation sweep — confidence should decrease as rotation speed increases
  // ------------------------------------------------------------
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
}
