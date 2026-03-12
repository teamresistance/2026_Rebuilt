package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShootingConstants;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

public class TurretConfidenceUtilTest {

  @Test
  void testConfidenceCalculation() {

    // Test parameters, change as needed to test different scenarios
    double vx = 1.0; // m/s
    double vy = 1.0; // m/s
    double rotationSpeed = 0.25; // rad/s
    double distance = 3.0; // meters

    // Mock drive subsystem
    SwerveDriveIO drive = mock(SwerveDriveIO.class);

    // Set up mock behavior for pose and speeds
    Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationSpeed);

    // Mock the drive subsystem to return the pose and speeds
    when(drive.getPose()).thenReturn(pose);
    when(drive.getChassisSpeedsFieldRelative()).thenReturn(speeds);

    // Mock static methods in ShootingUtil and ShootingConstants
    try (MockedStatic<ShootingUtil> shootingUtilMock = mockStatic(ShootingUtil.class);
        MockedStatic<ShootingConstants> shootingConstantsMock =
            mockStatic(ShootingConstants.class)) {

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
      shootingConstantsMock.when(() -> ShootingConstants.getTimeOfFlight(distance)).thenReturn(0.6);

      // Call the method under test
      double confidence = TurretConfidenceUtil.calculateConfidence(drive);

      // Print the confidence for debugging purposes
      System.out.println("Confidence = " + confidence);

      // Assert that the confidence is within the expected range (0 to 100)
      assertTrue(confidence > 0);
      assertTrue(confidence <= 100);
    }
  }
}
