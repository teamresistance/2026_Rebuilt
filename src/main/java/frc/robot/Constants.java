package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 *
 * <p>
 * NOTE: This class should only contain constants used by multiple subsystems.
 * Place
 * single-subsystem constants in the respective subsystem's folder.
 */
public final class Constants {

    public static boolean TEST_MODE = false;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean TUNING_MODE = false;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            5.0,
            Units.degreesToRadians(540),
            Units.degreesToRadians(400));

    public static final int CLIMBER_BRAKE_ID = 0; // TODO: correct DIO port
    public static final int CLIMBER_MOTOR_ID = 11;
    public static final int CLIMBER_ZERO = 0;
    public static final int CLIMBER_FULL = 10; // TODO: actual high/low pos

    public static final int SHOOTER_HOOD_ID = 12;
    public static final int SHOOTER_TURRET_ID = 13;
    public static final int SHOOTER_FLYWHEEL_ID = 14;
    public static final int SHOOTER_TURRET_ENCODER_ID = 15;

    // TODO: correct numbers with real hardware
    public static final double SHOOTER_HOOD_REVS_PER_DEG = 1;
    public static final double SHOOTER_TURRET_REVS_PER_DEG = 1;
    public static final double SHOOTER_HOOD_REVS_TOLERANCE = 0;
    public static final double SHOOTER_TURRET_REVS_TOLERANCE = 0;

    // TODO: correct numbers with real hardware
    public static final double SHOOTER_HOOD_MAX_PITCH = 47;
    public static final double SHOOTER_HOOD_MIN_PITCH = 13;
    public static final double SHOOTER_TURRET_MAX_YAW = 170;
    public static final double SHOOTER_TURRET_MIN_YAW = -170;

    // TODO: correct numbers with real hardware
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(0, 0, Rotation2d.kZero);
    public static final double SHOOTING_APPROXIMATE_TOF = 0.8;

    public enum LEDMode {
        RAINBOW,
        READY,
        SHOOTING,
        PASSING,
        NOT_READY,
        SHIFTING_US,
        SHIFTING_THEM,
        ENDGAME,
        BUMP
    }

    public static final int LED_START_INDEX = 0;
    public static final int LED_END_INDEX = 100; // todo: this

    public static final RainbowAnimation LED_ANIMATION_RAINBOW = new RainbowAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(60);
    public static final StrobeAnimation LED_ANIMATION_SHOOTING = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(100, 255, 100));
    public static final StrobeAnimation LED_ANIMATION_PASSING = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(100, 100, 255));
    public static final SolidColor LED_ANIMATION_READY = new SolidColor(LED_START_INDEX, LED_END_INDEX)
            .withColor(new RGBWColor(100, 255, 100));
    public static final SolidColor LED_ANIMATION_NOT_READY = new SolidColor(LED_START_INDEX, LED_END_INDEX)
            .withColor(new RGBWColor(255, 100, 100));
    public static final StrobeAnimation LED_ANIMATION_SHIFTING_US = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(255, 255, 0));
    public static final StrobeAnimation LED_ANIMATION_SHIFTING_THEM = new StrobeAnimation(LED_START_INDEX,
            LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(255, 155, 0));
    public static final StrobeAnimation LED_ANIMATION_ENDGAME = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(255, 100, 255));
    public static final StrobeAnimation LED_ANIMATION_BUMP = new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
            .withFrameRate(10)
            .withColor(new RGBWColor(0, 255, 255));

    public class ShootingConstants {

        /*
         * ===============
         * Ballistic Constants
         * ===============
         */

        /// Mass of the projectile fuel (kg) - affects drag and energy loss calculations
        public static final double FUEL_MASS = 0.227;

        // Vertical velocity component (m/s) - gravitational component of projectile
        // motion
        public static final double VERTICAL_VELOCITY_COMPONENT = 6.92858460962;

        // Square of vertical velocity component (m^2/s^2) - used in total velocity
        // calculations
        public static final double VERTICAL_VELOCITY_COMPONENT_SQUARED = 48.0052946927;

        // Drag coefficient - exponential factor in air resistance equation
        public static final double QUADRATIC_DRAG_COEFFICIENT = 0.004997;

        // Fixed flight time (seconds) - predetermined projectile flight duration used
        // in calculations
        public static final double MINIMUM_TIME_OF_FLIGHT = 1.15;

        /*
         * ===============
         * Motor Adjustment Constants
         * ===============
         */

        // Time interval for motor adjustment (seconds) - time to recover from launch
        // and prepare for next
        // shot
        public static final double RELOAD_TIME = 0.07675438596;

        // Upswing velocity of projectile before launch (m/s) - pre-launch speed
        // requirement for
        // consistent shooting performance
        public static final double UPSWING_VELOCITY = 5.7912;

        // Moment of inertia of the shooter mechanism (kg·m^2) - affects how much torque
        // is needed to
        // adjust wheel speed after a shot
        public static final double SHOOTER_MOMENT_OF_INERTIA = 0.00126;

        // Resting angular velocity of the shooter wheel (rad/s) - target steady-state
        // speed for
        // consistent shooting performance
        public static final double FREE_ANGULAR_VELOCITY = 0;

        /** Conversion factor from radians to degrees */
        public static final double RAD_TO_DEG = 180.0 / Math.PI;

        /** Conversion factor from degrees to radians */
        public static final double DEG_TO_RAD = Math.PI / 180.0;
    }

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
}
