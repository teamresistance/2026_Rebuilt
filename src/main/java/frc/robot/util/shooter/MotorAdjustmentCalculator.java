package frc.robot.util.shooter;

import frc.robot.ShootingConstants;


/**
 * MotorAdjustmentCalculator - Computes motor adjustments to compensate for energy loss during projectile launch.
 *
 * This class calculates the required angular acceleration of the launcher wheel needed to
 * maintain consistent performance after each projectile is fired, accounting for:
 * - Energy lost from fuel ejection
 * - Moment of inertia of the launcher mechanism
 * - Required upswing velocity before launch
 * - Resting angular velocity of the wheel
 *
 * The computation uses rotational dynamics equations to determine the motor acceleration
 * required to restore the wheel to its resting speed within a specified time interval.
 */
public class MotorAdjustmentCalculator {

    // Constants
    /** Moment of inertia of launcher wheel (kg·m²) - rotational mass property */
    private static final double I = ShootingConstants.SHOOTER_MOMENT_OF_INERTIA;
    /** Mass of projectile fuel (kg) - affects energy loss calculations */
    private static final double m = ShootingConstants.FUEL_MASS;

    /** Time interval for motor adjustment (seconds) - time to recover from launch */
    public static final double t = ShootingConstants.RELOAD_TIME;
    /** Upswing velocity of projectile before launch (m/s) - pre-launch speed requirement */
    private static final double v_up = ShootingConstants.UPSWING_VELOCITY;

    /** Resting angular velocity of launcher wheel (rad/s) - target steady-state speed */
    private static final double restingAngularVelocity = 0;

    /* =======================
     * Output State Variables
     * ======================= */

    /** Launch velocity of projectile (m/s) - received from ballistic calculator */
    private static double v_out = 0;

    /** Required angular acceleration of launcher motor (rad/s²) - compensates for energy loss */
    public static double desiredAngularAcceleration = 0;

    /** Desired angular velocity of launcher wheel after adjustment (rad/s) - target speed post-launch */
    public static double desiredAngularVelocity = 0;


    /**
     * Computes the required motor angular acceleration to restore launcher wheel speed.
     *
     * <p>This method calculates the angular acceleration needed to compensate for energy lost
     * when the projectile is launched. It retrieves the actual launch velocity from the ballistic
     * calculator, then uses rotational dynamics to determine the motor adjustment required to
     * return the wheel from its post-launch state back to resting angular velocity within the
     * specified time interval.
     *
     * <p>The computed angular acceleration is stored in {@link #desiredAngularAcceleration} and
     * relevant metrics are printed to standard output.
     */
    public static void computeMotorAdjustment() {
        v_out = FastBallisticCalculator.vTotalNew;

        long start = System.nanoTime();

        double desiredAngularVelocity = Math.sqrt(restingAngularVelocity * restingAngularVelocity + m / I * (v_out * v_out - v_up * v_up));

        desiredAngularAcceleration = (desiredAngularVelocity - restingAngularVelocity) / t;
        
        long end = System.nanoTime();

        System.out.println("Initial Angular Velocity: " + restingAngularVelocity);
        System.out.println("Desired Angular Acceleration: " + desiredAngularAcceleration);
        System.out.println("Desired Angular Velocity:" + desiredAngularVelocity);

        System.out.println("Launch Velocity: " + v_out);
        System.out.printf("time_s = %.9f%n", (end - start) * 1e-9);

    }
}