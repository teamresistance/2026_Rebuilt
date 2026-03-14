package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static boolean TEST_MODE = false;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final ShootingStyle CURRENT_SHOT_STYLE = ShootingStyle.CALC;
  public static final boolean TUNING_MODE = false;
  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          5.0,
          Units.degreesToRadians(540),
          Units.degreesToRadians(400));

  public static final int HOPPER_ROLLERS_ID = 18;
  public static final int HOPPER_WHEELS_ID = 17;
  public static final int TOWER_MOTOR_ID = 19;

  public static final int CLIMBER_MOTOR_ID = 11;
  public static final int CLIMBER_FULL_OUT = 0; // FULLY UP
  public static final int CLIMBER_FULL_IN = -140; // FULLY DOWN
  public static final int CLIMBER_BRAKE_RELAY_ID = 0; // TODO: me please!

  public static final int SHOOTER_HOOD_ID = 12;
  public static final int SHOOTER_TURRET_ID = 13;
  public static final int SHOOTER_FLYWHEEL_ID = 14;
  public static final int SHOOTER_FLYWHEEL_ID_2 = 15;

  public static final int INTAKE_MOTOR_ID = 16;

  public static final double SHOOTER_HOOD_REVS_PER_DEG = (4.0 * (175.0 / 10.0)) / 360.0;
  public static final double SHOOTER_TURRET_REVS_PER_DEG = (5 * 5 * 3.2) / 360.0;
  public static final double SHOOTER_HOOD_REVS_TOLERANCE = 0.15;
  public static final double SHOOTER_TURRET_REVS_TOLERANCE = 0.5;
  public static final double SHOOTER_RPS_TOLERANCE = 2;

  public static final double SHOOTER_HOOD_MAX_PITCH = 43;
  public static final double SHOOTER_HOOD_MIN_PITCH = 17.5;
  public static final double SHOOTER_TURRET_MAX_YAW = 135; // TODO: max min yaw
  public static final double SHOOTER_TURRET_MIN_YAW = -135;
  public static final double SHOOTER_TRIM_ADJUSTMENT_INCREMENT = 1; // TODO: number w/ driver

  public static final Transform2d ROBOT_TO_TURRET =
      new Transform2d(Units.inchesToMeters(1.38), Units.inchesToMeters(3.8), Rotation2d.kZero);

  public static final int CANDLE_ID = 40;

  public enum LEDMode {
    RAINBOW,
    SHOOTING_CONFIDENT,
    SHOOTING_DOUBTFUL,
    PASSING_CONFIDENT,
    PASSING_DOUBTFUL,
    INTAKING,
    ACTIVE,
    INACTIVE,
    BUMP,
    AUTO
  }

  public static final int LED_START_INDEX = 0;
  public static final int LED_END_INDEX = 100; // todo: this

  public static final RainbowAnimation LED_ANIMATION_RAINBOW =
      new RainbowAnimation(LED_START_INDEX, LED_END_INDEX).withFrameRate(60);
  public static final StrobeAnimation LED_ANIMATION_SHOOTING_CONFIDENT =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(50, 255, 50));
  public static final StrobeAnimation LED_ANIMATION_SHOOTING_DOUBTFUL =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 50, 50));
  public static final StrobeAnimation LED_ANIMATION_PASSING_CONFIDENT =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(50, 50, 255));
  public static final StrobeAnimation LED_ANIMATION_PASSING_DOUBTFUL =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 50, 50));
  public static final StrobeAnimation LED_ANIMATION_INTAKING =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(255, 200, 0))
          .withFrameRate(5);
  public static final SolidColor LED_ANIMATION_ACTIVE =
      new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 0, 255));
  public static final LarsonAnimation LED_ANIMATION_INACTIVE =
      new LarsonAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(0, 100, 0))
          .withFrameRate(8)
          .withSize(6);
  public static final StrobeAnimation LED_ANIMATION_BUMP =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withFrameRate(10)
          .withColor(new RGBWColor(0, 255, 255));
  public static final LarsonAnimation LED_ANIMATION_AUTO =
      new LarsonAnimation(LED_START_INDEX, LED_END_INDEX)
          .withFrameRate(10)
          .withColor(new RGBWColor(255, 150, 0))
          .withSize(10);

  // at what confidence is it considered "confident" instead of "doubtful" TODO: me
  public static final double CONFIDENCE_THRESHOLD = 0.5;

  public enum ShiftOwner {
    BLUE,
    RED,
    BOTH,
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum ShootingStyle {
    /** Using interpolation maps for shooting */
    MAPS,

    /** Using drag calculations for shooting */
    CALC
  }

  public class ShootingConstants {

    private ShootingConstants() {
      throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    // Mass of the projectile fuel (kg) - affects drag and energy loss calculations
    public static final double FUEL_MASS = 0.227;

    // Hub height (m) specifically to the top of the polycarbonate edges
    public static final double HUB_HEIGHT = 1.8288;

    // Turret height (m)
    public static final double TURRET_HEIGHT = 0.7112;

    // Acceleration due to gravity (m/s^2)
    public static final double G = 9.80665;

    // Fixed flight time (seconds) - predetermined projectile flight duration used in calculations
    public static final double MINIMUM_TIME_OF_FLIGHT = 1.2;

    // Drag coefficient - exponential factor in air resistance equation
    public static final double QUADRATIC_DRAG_COEFFICIENT =
        0.0429140552109; // theoretical value is 0.004997, empirically tested value is
    // 0.0429140552109

    private static final double ROOT_MG_OVER_K =
        Math.sqrt(FUEL_MASS * G / QUADRATIC_DRAG_COEFFICIENT);

    public static final double VERTICAL_VELOCITY_COMPONENT =
        ROOT_MG_OVER_K
            * (Math.exp(QUADRATIC_DRAG_COEFFICIENT * (HUB_HEIGHT - TURRET_HEIGHT) / FUEL_MASS)
                - Math.cos((1 / ROOT_MG_OVER_K) * G * MINIMUM_TIME_OF_FLIGHT))
            / (Math.sin((1 / ROOT_MG_OVER_K) * G * MINIMUM_TIME_OF_FLIGHT));

    /*
     * ===============
     * Motor Adjustment Constants
     * ===============
     */

    // TODO: correct numbers with real hardware
    // Time interval for motor adjustment (seconds) - time to recover from launch
    // and prepare for next
    // shot
    public static final double RELOAD_TIME = 0.07675438596;

    public static final double SHOOTER_RADIUS = 0.0381; // (meters)
    public static final double SHOOTER_EFFICIENCY =
        0.60; // worst-case placeholder. TODO: empirically calculate

    /** Conversion factor from radians to degrees */
    public static final double RAD_TO_DEG = 180.0 / Math.PI;

    /** Conversion factor from degrees to radians */
    public static final double DEG_TO_RAD = Math.PI / 180.0;
  }
}
