package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
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

  public static final int HOPPER_ROLLERS_ID = 18;
  public static final int HOPPER_WHEELS_ID = 17;
  public static final int TOWER_MOTOR_ID = 19;

  public static final int CLIMBER_MOTOR_ID = 11;
  public static final int CLIMBER_FULL_OUT = 0; // FULLY UP
  public static final int CLIMBER_FULL_IN = -138; // FULLY DOWN
  public static final int CLIMBER_BRAKE_RELAY_ID = 1;

  public static final int SHOOTER_HOOD_ID = 12;
  public static final int SHOOTER_TURRET_ID = 13;
  public static final int SHOOTER_FLYWHEEL_ID = 14;
  public static final int SHOOTER_FLYWHEEL_ID_2 = 15;

  public static final int INTAKE_MOTOR_ID = 16;

  public static final double SHOOTER_HOOD_REVS_PER_DEG = (4.0 * (175.0 / 10.0)) / 360.0;
  public static final double SHOOTER_TURRET_REVS_PER_DEG = (5 * 5 * 3.2) / 360.0;
  public static final double SHOOTER_HOOD_REVS_TOLERANCE = 3;
  public static final double SHOOTER_TURRET_REVS_TOLERANCE = 3;
  public static final double SHOOTER_RPS_TOLERANCE = 0.5;

  public static final double SHOOTER_HOOD_MAX_PITCH = 43;
  public static final double SHOOTER_HOOD_MIN_PITCH = 17.5;
  public static final double SHOOTER_TURRET_MAX_YAW = 129;
  public static final double SHOOTER_TURRET_MIN_YAW = -129;
  public static final double SHOOTER_TRIM_ADJUSTMENT_INCREMENT = 3;

  public static final Transform2d ROBOT_TO_TURRET =
      new Transform2d(Units.inchesToMeters(3.5), Units.inchesToMeters(0.75), Rotation2d.kZero);

  // z = 0 because it gets overridden by limelight height
  public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(0); // TODO
  public static final Transform3d TURRET_TO_LIMELIGHT =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-1.215), Units.inchesToMeters(-6.248), 0.0),
          new Rotation3d(0, Units.degreesToRadians(25.0), 0));

  public static final Translation2d CENTER_OF_ROTATION =
      new Translation2d(ROBOT_TO_TURRET.getX(), ROBOT_TO_TURRET.getY());

  public static final int LED_CANDLE_ID = 40;
  public static final CANBus LED_CANDLE_BUS = new CANBus("drive");

  public enum LED_MODE {
    ACTIVE,
    INACTIVE,
    SHIFT_WARNING,
    SHIFT_SWITCHING_RED,
    SHIFT_SWITCHING_GREEN,
    ENDGAME_WARNING,
    CLIMB_WARNING,
    AUTO,
    DISABLED_DIM
  }

  public static final int LED_START_INDEX = 0;
  public static final int LED_END_INDEX = 161;

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
}
