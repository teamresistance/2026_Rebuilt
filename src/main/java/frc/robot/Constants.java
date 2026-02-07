package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
  public static final boolean TUNING_MODE = false;
  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          5.0,
          Units.degreesToRadians(540),
          Units.degreesToRadians(400));

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

  public enum ShootingTarget {
    HUB,
    TOP_ZONE,
    BOTTOM_ZONE,
  }

  public enum LEDMode {
    RAINBOW,
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
