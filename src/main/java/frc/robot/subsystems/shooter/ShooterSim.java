package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.FieldConstants;
import frc.robot.util.shooter.ShootingManager;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSim implements ShooterIO {

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d turretBase;
  private final MechanismLigament2d hood;

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

  public ShooterSim() {
    register();

    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("turretRoot", 1.5, 0.5);

    turretBase =
        root.append(new MechanismLigament2d("TurretBase", 0.6, 0, 12, new Color8Bit(Color.kBlue)));

    hood = turretBase.append(new MechanismLigament2d("Hood", 0.4, 0, 6, new Color8Bit(Color.kRed)));
    SmartDashboard.putData("Turret Mechanism2d", mech);
  }

  /**
   * Set suppliers that provide the current robot pose and chassis speeds from the drive subsystem.
   * RobotContainer should call this when running in simulation so ShootingManager can be fed with
   * real sim data.
   */
  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
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
    hood.setAngle(hoodAngleDegs);

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
        // between the vector to the hub and the vector to the hub's projection on
        // the alliance wall (three-point angle: hub, robot, hubProjection).
        var targetPose = ShootingManager.SimulationAndState.getShootingTarget(pose);

        Translation2d hub = targetPose.getTranslation();
        Translation2d robot = pose.getTranslation();

        // Distance from robot to hub
        double dx = hub.getX() - robot.getX();
        double dy = hub.getY() - robot.getY();
        double distanceToHub = Math.hypot(dx, dy);

        // Determine alliance wall X coordinate. Use known apriltag X positions as
        // field edge proxies (left = APRILTAG_29.x, right = APRILTAG_13.x).
        double wallX = hub.getX();
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent()) {
          if (allianceOpt.get() == DriverStation.Alliance.Blue) {
            wallX = FieldConstants.APRILTAG_29.getX();
          } else {
            wallX = FieldConstants.APRILTAG_13.getX();
          }
        }

        // Project the hub onto the alliance wall (preserve Y, replace X with wall X)
        Translation2d hubProj = new Translation2d(wallX, hub.getY());

        // Vectors from robot to hub and robot to projection
        double v1x = dx; // hub - robot
        double v1y = dy;
        double v2x = hubProj.getX() - robot.getX(); // proj - robot
        double v2y = hubProj.getY() - robot.getY();

        double dot = v1x * v2x + v1y * v2y;
        double cross = v1x * v2y - v1y * v2x;

        double denom = Math.hypot(v1x, v1y) * Math.hypot(v2x, v2y);
        double fieldRelativeAngleToHub = 0.0;
        if (denom > 1e-9) {
          // Signed angle between v1 and v2
          fieldRelativeAngleToHub = Math.atan2(cross, dot);
        }

        ShootingManager.updateShootingParameters(distanceToHub, fieldRelativeAngleToHub, speeds);
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
  public void runFlywheelAtRPS(double rps) {
    // not relevant in sim
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
