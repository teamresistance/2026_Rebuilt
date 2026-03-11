package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class TurretConfidenceUtilTest {

  // Editable parameters for experimentation and testing
  double stdDev = 1.0;

  double distance = 3.0;

  double vx = 1.0;
  double vy = 1.0;
  double rotationspeed = 0.25;

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
  @DisplayName("Tested situation shot has high confidence")
  public void testedSituationShot() {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);

    System.out.println("Tested situation confidence = " + confidence);

    assertTrue(
        confidence > 50, "Expected confidence > 50% for tested situation, but got " + confidence);
  }

  @ParameterizedTest
  @DisplayName("X velocity sweep")
  @ValueSource(doubles = {0}) // typical sweep: 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5
  void xVelocitySweep(double vx) {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);

    System.out.println("vx=" + vx + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  @ParameterizedTest
  @DisplayName("Y velocity sweep")
  @ValueSource(doubles = {0}) // typical sweep: 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5
  void yVelocitySweep(double vy) {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);
    System.out.println("vy=" + vy + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  @ParameterizedTest
  @DisplayName("Rotation speed sweep")
  @ValueSource(
      doubles = {0}) // typical sweep: 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5
  void rotationSpeedSweep(double rotationspeed) {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);
    System.out.println("rotationspeed=" + rotationspeed + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  @ParameterizedTest
  @DisplayName("Distance sweep")
  @ValueSource(doubles = {0}) // typical sweep: 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5
  void distanceSweep(double distance) {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);
    System.out.println("distance=" + distance + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }

  @ParameterizedTest
  @DisplayName("Standard deviation sweep")
  @ValueSource(doubles = {1}) // typical sweep: 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5
  void stdDevSweep(double stdDev) {

    double confidence = calculateConfidence(distance, vx, vy, rotationspeed, stdDev);
    System.out.println("stdDev=" + stdDev + " confidence=" + confidence);

    assertTrue(confidence > 0);
  }
}
