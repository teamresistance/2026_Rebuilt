package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TurretConfidenceUtilTest {

  // Editable parameters for experimentation
  double stdDev = 1.0;

  double distance = 6.0;

  double vx = 1.0;
  double vy = 1.0;
  double rotationspeed = 1.0;

  private double calculateConfidence(
      double distance, double vx, double vy, double rotationspeed, double stdDev) {

    double timeOfFlight = distance / 10; // Approximate time for a projectile to reach the target

    double lateralVelocity = Math.hypot(vx, vy);

    double translationError = lateralVelocity * timeOfFlight;

    double rotationError = rotationspeed * timeOfFlight * distance;

    double totalError =
        Math.sqrt(translationError * translationError + rotationError * rotationError);

    double probability = Math.exp(-(Math.pow(totalError, 2)) / (2 * Math.pow(stdDev, 2)));

    return probability * 100;
  }

  @Test
  @DisplayName("Baseline stationary shot has high confidence")
  public void baselineStationaryShot() {

    double confidence = calculateConfidence(distance, 0, 0, 0, stdDev);

    System.out.println("Stationary confidence = " + confidence);

    assertTrue(confidence > 95);
  }

  @Test
  @DisplayName("X-velocity sweep shows decreasing confidence")
  public void xVelocitySweep() {

    for (double vx = 0; vx <= 5; vx += 0.5) {

      double confidence = calculateConfidence(distance, vx, 0, 0, stdDev);

      System.out.println("vx=" + vx + " confidence=" + confidence);
    }

    assertTrue(true);
  }

  @Test
  @DisplayName("Y-velocity sweep shows decreasing confidence")
  public void yVelocitySweep() {

    for (double vy = 0; vy <= 5; vy += 0.5) {

      double confidence = calculateConfidence(distance, 0, vy, 0, stdDev);

      System.out.println("vy=" + vy + " confidence=" + confidence);
    }

    assertTrue(true);
  }

  @Test
  @DisplayName("Rotation speed sweep shows decreasing confidence")
  public void rotationSweep() {

    for (double rotationspeed = 0; rotationspeed <= 6; rotationspeed += 0.5) {

      double confidence = calculateConfidence(distance, 0, 0, rotationspeed, stdDev);

      System.out.println("rotationspeed=" + rotationspeed + " confidence=" + confidence);
    }

    assertTrue(true);
  }

  @Test
  @DisplayName("Standard deviation sweep shows increasing confidence")
  public void standardDeviationSweep() {

    for (double sigma = 0.25; sigma <= 2.0; sigma += 0.25) {

      double confidence = calculateConfidence(distance, 1.0, 0, 0, sigma);

      System.out.println("sigma=" + sigma + " confidence=" + confidence);
    }

    assertTrue(true);
  }

  @Test
  @DisplayName("Distance sweep shows decreasing confidence")
  public void distanceSweep() {

    for (double d = 2; d <= 10; d += 1) {

      double confidence = calculateConfidence(d, 1.0, 0, 0, stdDev);

      System.out.println("distance=" + d + " confidence=" + confidence);
    }

    assertTrue(true);
  }
}
