package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.util.FastBallisticCalculator;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {

  private final SwerveDriveIO drive;
  private final ShooterIO shooter;

  public IdleShooterCommand(SwerveDriveIO _drive, ShooterIO _shooter) {
    drive = _drive;
    shooter = _shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    // 1. Get current robot state and shooter angles (Field-Relative)
    Pose2d robotPose = drive.getPose();
    double hoodAngleRad = Math.toRadians(ShooterIO.getVerticalShootingAngle());
    double turretAngleRad = Math.toRadians(ShooterIO.getHorizontalTotalShootingAngle());
    double launchSpeed = ShooterIO.getLaunchVelocity();

    // 2. Calculate horizontal velocity relative to the robot
    double vHorizontalRelative = launchSpeed * Math.cos(hoodAngleRad);
    double vxRelative = vHorizontalRelative * Math.cos(turretAngleRad);
    double vyRelative = vHorizontalRelative * Math.sin(turretAngleRad);

    // 3. Add robot's field-relative velocity to get the true initial floor velocity
    // (Assuming drive.getVelocity() returns a Transform2d representing velocity in m/s)
    double vxRobot = drive.getVelocity().getX();
    double vyRobot = drive.getVelocity().getY();

    double vxFloor = vxRelative + vxRobot;
    double vyFloor = vyRelative + vyRobot;

    // Total initial horizontal speed and direction relative to the floor
    double vFloorInitial = Math.sqrt(vxFloor * vxFloor + vyFloor * vyFloor);
    double floorAzimuth = Math.atan2(vyFloor, vxFloor);

    // 4. Apply 1D Analytic Quadratic Drag to find actual travel distance
    // Derivation of velocity decay integrated over time: D = (M / K) * ln(1 + (K / M) * v0 * T)
    double m = FastBallisticCalculator.M;
    double k = ShootingConstants.QUADRATIC_DRAG_COEFFICIENT;
    double t = ShootingConstants.MINIMUM_TIME_OF_FLIGHT;

    // This prevents division-by-zero or log errors if the shooter is off/idle
    double actualTravelDistance = 0.0;
    if (vFloorInitial > 0.1) {
      actualTravelDistance = (m / k) * Math.log(1.0 + (k / m) * vFloorInitial * t);
    }

    // 5. Calculate final predicted landing pose
    Pose2d landingAt =
        new Pose2d(
            robotPose.getX() + actualTravelDistance * Math.cos(floorAzimuth),
            robotPose.getY() + actualTravelDistance * Math.sin(floorAzimuth),
            Rotation2d.fromRadians(floorAzimuth));

    // Log the corrected predicted landing position
    Logger.recordOutput("Shooter/LandingAt", landingAt);

    // Log the raw turret direction for debugging aim
    Pose2d lookingAt =
        new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(turretAngleRad));
    Logger.recordOutput("Shooter/AimingAt", lookingAt);

    // Set hardware targets
    double distance = ShooterIO.getPredictedDistanceToHubAfterReload();
    shooter.setTurretTarget(Math.toDegrees(turretAngleRad));
    shooter.setHoodTarget(Math.toDegrees(hoodAngleRad));

    Logger.recordOutput("Shooter/Virtual Distance to Hub", distance);
  }
}
