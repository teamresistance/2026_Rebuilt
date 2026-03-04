package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.shooter.ShootingPredictions;
import frc.robot.Constants.ShootingStyle;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSimCalc implements ShooterIO {

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d turretBase;
  // Visualization: target/angles and velocities (with visible labels)
  private final MechanismLigament2d totalHorizontal; // total horizontal shooting angle
  private final MechanismLigament2d originalHorizontal; // original angle to hub
  private final MechanismLigament2d robotVel;
  private final MechanismLigament2d shotVel;
  private final MechanismLigament2d hood;

  private double turretAngleDegs = 180;
  private double hoodAngleDegs = 13;
  private double turretTargetDegs = 180.0;
  private double hoodTargetDegs = 13.0;
  Transform2d acceleration = new Transform2d();

  private final PIDController turretPID = new PIDController(0.5, 0.0, 0);
  private final PIDController hoodPID = new PIDController(0.8, 0.0, 0);

  private final ShootingStyle calcMode;

  // Optional suppliers to obtain real simulation pose and chassis speeds from the
  // drive subsystem. When set by RobotContainer (in SIM mode), these will be used
  // to provide real simulation inputs to the ShootingManager.
  private Supplier<Pose2d> poseSupplier = null;
  private Supplier<ChassisSpeeds> speedsSupplier = null;

  public ShooterSimCalc(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, ShootingStyle calcMode) {
    register();

    this.calcMode = calcMode;
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

    hood = turretBase.append(new MechanismLigament2d("Hood", 0.4, 0, 6, new Color8Bit(Color.kRed)));
    SmartDashboard.putData("Turret Mechanism2d", mech);
  }

  // Keep turret/robot angles continuous across 0/360 boundary
  private double lastOriginalAngleDeg = 0;

  private double unwrapAngle(double angleDeg) {
    double delta = angleDeg - lastOriginalAngleDeg;
    if (delta > 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    lastOriginalAngleDeg += delta;
    return lastOriginalAngleDeg;
  }

  @Override
  public void periodic() {

    double turretOutput;
    double hoodOutput;

    switch (calcMode) {
      case MAPS:
        turretOutput = turretPID.calculate(turretAngleDegs, turretTargetDegs);
    hoodOutput = hoodPID.calculate(hoodAngleDegs, hoodTargetDegs);

    turretOutput = MathUtil.clamp(turretOutput, -3.0, 3.0);
    hoodOutput = MathUtil.clamp(hoodOutput, -2.0, 2.0);

    turretAngleDegs += turretOutput;
    hoodAngleDegs += hoodOutput;

    turretBase.setAngle(turretAngleDegs);
    hood.setAngle(hoodAngleDegs);
    break;
    case CALC:
      // Compute shortest-path turret rotation
    double error = angleDifferenceDeg(turretTargetDegs, turretAngleDegs);
    turretOutput = turretPID.calculate(0, error); // PID expects setpoint=0 for error
    hoodOutput = hoodPID.calculate(hoodAngleDegs, hoodTargetDegs);
    // acceleration = ;

    turretOutput = MathUtil.clamp(turretOutput, -3.0, 3.0);
    hoodOutput = MathUtil.clamp(hoodOutput, -2.0, 2.0);

    turretAngleDegs += turretOutput;
    hoodAngleDegs += hoodOutput;

    // Wrap turret angle safely to [0,360) for visualization and consistency
    turretAngleDegs = MathUtil.inputModulus(turretAngleDegs, 0.0, 360.0);

    turretBase.setAngle(turretAngleDegs);

    // Wrap turret angle to [0,360) for visualization and consistency
    turretAngleDegs = (turretAngleDegs + 360.0) % 360.0;

    turretBase.setAngle(turretAngleDegs);

    // Existing simulation visualization updates remain unchanged
    if (poseSupplier != null && speedsSupplier != null) {
      try {
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speeds = speedsSupplier.get();
        double fieldRelativeAngleToHub = calculator.getPredictedFieldRelativeAngleToHubAfterReload();
        double totalHorizDeg = MathUtil.inputModulus(calculator.getHorizontalTotalShootingAngle(), 0.0, 360.0);
        totalHorizontal.setAngle(totalHorizDeg);

        double rawAngleDeg = Math.toDegrees(fieldRelativeAngleToHub);
        double smoothAngleDeg = unwrapAngle(rawAngleDeg);
        originalHorizontal.setAngle(smoothAngleDeg);

        var robotRotation = pose.getRotation();
        double cosR = Math.cos(robotRotation.getRadians());
        double sinR = Math.sin(robotRotation.getRadians());
        double vxField = speeds.vxMetersPerSecond * cosR - speeds.vyMetersPerSecond * sinR;
        double vyField = speeds.vxMetersPerSecond * sinR + speeds.vyMetersPerSecond * cosR;

        double robotSpeed = Math.hypot(vxField, vyField);
        double robotAngleDeg = (Math.toDegrees(Math.atan2(vyField, vxField)) + 360) % 360;
        robotVel.setAngle(robotAngleDeg);
        robotVel.setLength(Math.min(1.5, robotSpeed * 0.25));

        double launch = calculator.getLaunchVelocity();
        shotVel.setAngle(calculator.getHorizontalTotalShootingAngle());
        shotVel.setLength(Math.min(1.5, Math.abs(launch) * 0.075));
      } catch (Exception ex) {
        Logger.recordOutput("Shooter/Sim Error", ex.toString());
      }
      break;
    }

    }

    Logger.recordOutput("Shooter/Sim Turret Angle", turretAngleDegs);
    Logger.recordOutput("Shooter/Sim Hood Angle", hoodAngleDegs);
    Logger.recordOutput("Shooter/Sim Turret Target", turretTargetDegs);
    Logger.recordOutput("Shooter/Sim Hood Target", hoodTargetDegs);
  }

  /** Computes the shortest angular difference from current to target in degrees [-180,180] */
  private double angleDifferenceDeg(double target, double current) {
    return MathUtil.inputModulus(target - current, -180.0, 180.0);
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
