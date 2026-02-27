package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.SimulationAndState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSim extends ShooterIO {

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d turretBase;
  // Visualization: target/angles and velocities (with visible labels)
  private final MechanismLigament2d totalHorizontal; // total horizontal shooting angle
  private final MechanismLigament2d originalHorizontal; // original angle to hub
  private final MechanismLigament2d robotVel;
  private final MechanismLigament2d shotVel;

  private double turretAngleDegs = 180;
  private double hoodAngleDegs = 13;
  private double turretTargetDegs = 180.0;
  private double hoodTargetDegs = 13.0;

  private final PIDController turretPID = new PIDController(0.5, 0.0, 0);
  private final PIDController hoodPID = new PIDController(0.8, 0.0, 0);

  // Optional suppliers to obtain real simulation pose and chassis speeds from the
  // drive subsystem. When set by RobotContainer (in SIM mode), these will be used
  // to provide real simulation inputs to the ShootingManager.
  private Supplier<Pose2d> poseSupplier = null;
  private Supplier<ChassisSpeeds> speedsSupplier = null;

  public ShooterSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    register();

    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;

    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("turretRoot", 1.5, 0.5);

    turretBase =
        root.append(new MechanismLigament2d("TurretBase", 0.6, 0, 12, new Color8Bit(Color.kBlack)));

    // Show the total horizontal shooting angle (green). This is the absolute
    // horizontal angle the shooter should be set to in the faield frame.
    totalHorizontal =
        root.append(
            new MechanismLigament2d(
                "Total Horizontal (deg)", 0.6, turretTargetDegs, 4, new Color8Bit(Color.kGreen)));

    // Show the original field-relative angle to the hub (blue-ish) so we can
    // compare current target vs raw angle-to-hub.
    originalHorizontal =
        root.append(
            new MechanismLigament2d(
                "Original Horizontal (deg)", 0.6, 0, 3, new Color8Bit(Color.kTeal)));

    // Robot velocity vector (cyan) anchored at the root
    robotVel =
        root.append(new MechanismLigament2d("RobotVel", 0.6, 0, 3, new Color8Bit(Color.kCyan)));

    // Shot velocity vector (magenta/cyan mix) anchored at turret tip
    shotVel =
        turretBase.append(
            new MechanismLigament2d("ShotVel", 0.8, 0, 5, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("Turret Mechanism2d", mech);
  }

  @Override
  public void periodic() {
    double turretOutput = turretPID.calculate(turretAngleDegs, turretTargetDegs);
    double hoodOutput = hoodPID.calculate(hoodAngleDegs, hoodTargetDegs);

    turretOutput = MathUtil.clamp(turretOutput, -3.0, 3.0);
    hoodOutput = MathUtil.clamp(hoodOutput, -2.0, 2.0);

    turretAngleDegs += turretOutput;
    hoodAngleDegs += hoodOutput;

    turretBase.setAngle(turretAngleDegs);
    ;

    Logger.recordOutput("Shooter/Sim Turret Angle", turretAngleDegs);
    Logger.recordOutput("Shooter/Sim Hood Angle", hoodAngleDegs);
    Logger.recordOutput("Shooter/Sim Turret Target", turretTargetDegs);
    Logger.recordOutput("Shooter/Sim Hood Target", hoodTargetDegs);

    // If simulation data suppliers are available, use them to update shooting
    // parameters with real sim pose and chassis speeds. This drives the
    // ShootingManager with the same inputs the real robot code would use.
    if (poseSupplier != null && speedsSupplier != null) {
      try {
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speeds = speedsSupplier.get();

        // Determine the target pose based on the current robot pose and alliance
        // settings. Compute the straight-line distance to the hub and the angle
        // between the vector to the hub and the +X direction at the robot
        // (i.e., the three points: hub, robot, robot+(1,0)).
        var targetPose = SimulationAndState.getShootingTarget(pose);

        Translation2d hub = targetPose.getTranslation();
        Translation2d robot = pose.getTranslation();

        // Distance from robot to hub
        double dx = hub.getX() - robot.getX();
        double dy = hub.getY() - robot.getY();
        double distanceToHub = Math.hypot(dx, dy);

        // Angle between vector robot->hub and robot->(robot + (1,0)). For the
        // +X direction the second vector is (1,0), so the signed angle is
        // atan2(cross, dot) = atan2(-dy, dx) == -atan2(dy, dx).
        double fieldRelativeAngleToHub = 0.0;
        double denom = Math.hypot(dx, dy);
        if (denom > 1e-9) {
          fieldRelativeAngleToHub = Math.atan2(dy, dx);
        }

        // Record the computed inputs so we can trace where a zero angle might
        // originate (helps debugging in simulation/AdvantageScope).
        Logger.recordOutput("Shooter/Sim Input DistanceToHub", distanceToHub);
        Logger.recordOutput(
            "Shooter/Sim InputFieldRelativeAngleToHub", Math.toDegrees(fieldRelativeAngleToHub));

        // Update the ShootingManager with the latest inputs so ballistic
        // parameters are available for visualization below.
        updateShootingParameters(distanceToHub, fieldRelativeAngleToHub, speeds, pose);

        // --- Visualization updates ---
        // Show the desired absolute shooting angle computed by the manager
        // (total horizontal) and the original raw angle to the hub so you can
        // visually compare them.
        totalHorizontal.setAngle(getHorizontalTotalShootingAngle());

        // original angle to hub (field-relative) in degrees
        originalHorizontal.setAngle(Math.toDegrees(fieldRelativeAngleToHub));

        // Compute robot field-frame velocity (match ShootingManager conversion).
        var robotRotation = pose.getRotation();
        double cosR = Math.cos(robotRotation.getRadians());
        double sinR = Math.sin(robotRotation.getRadians());
        double vxField = speeds.vxMetersPerSecond * cosR - speeds.vyMetersPerSecond * sinR;
        double vyField = speeds.vxMetersPerSecond * sinR + speeds.vyMetersPerSecond * cosR;

        double robotSpeed = Math.hypot(vxField, vyField);
        double robotAngleDeg = Math.toDegrees(Math.atan2(vyField, vxField));
        robotVel.setAngle(robotAngleDeg);
        // Scale the length for visualization (clamp to a reasonable size)
        robotVel.setLength(Math.min(1.5, robotSpeed * 0.25));

        // Shot velocity vector: angle uses the manager's total horizontal angle,
        // length is proportional to computed launch velocity.
        double launch = getLaunchVelocity();
        shotVel.setAngle(getHorizontalTotalShootingAngle());
        shotVel.setLength(Math.min(1.5, Math.abs(launch) * 0.075));
      } catch (Exception ex) {
        // Defensive: do not let simulation UI break on unexpected errors. Log and continue.
        Logger.recordOutput("Shooter/Sim Error", ex.toString());
      }
    }
  }

  @Override
  public boolean atShootingSetpoints() {
    return turretPID.atSetpoint() && hoodPID.atSetpoint();
  }

  @Override
  public boolean atTargetRPS() {
    return true;
  }

  @Override
  public void runFlywheelAtRPS(double rps) {
    // not relevant in sim
  }

  public boolean isShooting() {
    return true;
  }

  @Override
  public void setTurretTarget(double turretTargetAngle) {
    turretTargetDegs = turretTargetAngle;
  }

  @Override
  public void setHoodTarget(double hoodTargetAngle) {
    hoodTargetDegs = hoodTargetAngle;
  }

  @Override
  public void zeroHood(double newValue) {
    // not relevant in sim
  }
}
