package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

public class MotorTemperatureChecker {

  public enum TempState {
    NORMAL,
    WARNING,
    CRITICAL
  }

  private final double warningTempC;
  private final double criticalTempC;

  public MotorTemperatureChecker(double warningTempC, double criticalTempC) {
    this.warningTempC = warningTempC;
    this.criticalTempC = criticalTempC;
  }

  public TempState check(TalonFX motor) {
    double temp = motor.getDeviceTemp().getValueAsDouble();

    if (temp >= criticalTempC) return TempState.CRITICAL;
    if (temp >= warningTempC) return TempState.WARNING;
    return TempState.NORMAL;
  }

  public double getTemp(TalonFX motor) {
    return motor.getDeviceTemp().getValueAsDouble();
  }
}
