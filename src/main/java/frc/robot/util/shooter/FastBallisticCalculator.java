package frc.robot.util.shooter;

import frc.robot.ShootingConstants;

/**
 * FastBallisticCalculator - Calculates projectile launch parameters with air resistance.
 *
 * This class computes the required launch velocity and angle for a projectile to hit a target
 * at a specified distance, accounting for:
 * - Exponential air drag force proportional to distance traveled
 * - Vertical gravitational component
 * - Robot platform velocity offset (converts target frame to robot frame)
 *
 * The computation uses physics equations for projectile motion with quadratic drag,
 * solving for launch velocity magnitude and direction corrections relative to the robot's velocity.
 * -Samuel Sapatla, Mihir Sonthi
 */
public final class FastBallisticCalculator {

    // Constants
    /** Vertical velocity component (m/s) - gravitational component of projectile */
    private static final double VY = ShootingConstants.VERTICAL_VELOCITY_COMPONENT;
    /** Square of vertical velocity component (VY^2) - used for total velocity calculations */
    private static final double VY2 = ShootingConstants.VERTICAL_VELOCITY_COMPONENT_SQUARED;
    /** Mass coefficient (kg) - affects drag acceleration calculations */
    public static final double M = ShootingConstants.FUEL_MASS;
    /** Drag coefficient - exponential factor in air resistance equation */
    private static final double K = ShootingConstants.QUADRATIC_DRAG_COEFFICIENT;
    /** Fixed flight time (seconds) - predetermined projectile flight duration */
    private static final double T = ShootingConstants.MINIMUM_TIME_OF_FLIGHT;

    /* =======================
     * Output State Variables
     * ======================= */

    /** Final total launch velocity magnitude (m/s) */
    public static double vTotalNew;

    /** Vertical elevation angle above the floor (degrees) */
    public static double thetaDeg;

    /** Required correction to the floor azimuth (degrees) */
    public static double deltaFloorAngleDeg;

    /**
     * Computes the required projectile launch parameters while compensating for robot motion.
     *
     * <p>This method performs all calculations in the floor (field) reference frame. It first
     * computes the required floor-parallel projectile velocity assuming a stationary robot,
     * then applies a Galilean transformation by subtracting the robot's floor velocity. From the
     * corrected velocity vector, it computes the required total launch speed and azimuth correction.
     *
     * <p>The computed values are stored in static state variables and printed to standard output.
     *
     * @param D distance to the target along the floor (meters)
     * @param floorAzimuthDeg absolute azimuth angle of the target in the floor frame (degrees)
     * @param v_x robot floor velocity in the X direction (m/s)
     * @param v_y robot floor velocity in the Y direction (m/s)
     */
    public static void computeBallistics(
            double D,
            double floorAzimuthDeg,
            double v_x,
            double v_y
    ) {
        long start = System.nanoTime();

        // Base floor-parallel shoot speed
        double expTerm = Math.exp((K * D) / M);
        double vFloor = (M * (expTerm - 1.0)) / (K * T);

        // Vertical elevation angle
        thetaDeg = Math.atan(VY / vFloor) * ShootingConstants.RAD_TO_DEG;

        // Original floor-frame shoot vector
        double azRad = floorAzimuthDeg * ShootingConstants.DEG_TO_RAD;
        double vx0 = vFloor * Math.cos(azRad);
        double vy0 = vFloor * Math.sin(azRad);

        // Subtract robot floor velocity
        double vx1 = vx0 - v_x;
        double vy1 = vy0 - v_y;

        // New floor-parallel magnitude
        double vFloorNew = Math.sqrt(vx1 * vx1 + vy1 * vy1);

        // New total shoot velocity
        vTotalNew = Math.sqrt(VY2 + vFloorNew * vFloorNew);

        // Floor azimuth correction (floor-frame)
        double cross = vx0 * vy1 - vy0 * vx1;
        double dot   = vx0 * vx1 + vy0 * vy1;
        deltaFloorAngleDeg = Math.atan2(cross, dot) * ShootingConstants.RAD_TO_DEG;

        long end = System.nanoTime();

        // Output
        System.out.println("v_total = " + vTotalNew);
        System.out.println("theta_deg = " + thetaDeg);
        System.out.println("floor_angle_delta_deg = " + deltaFloorAngleDeg);
        System.out.printf("time_s = %.9f%n", (end - start) * 1e-9);
    }
}
