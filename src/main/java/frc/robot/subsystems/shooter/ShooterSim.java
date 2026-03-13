package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterSim implements ShooterIO {

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d turretBase;
  private final MechanismLigament2d hood;

  private double turretAngleDegs = 0;
  private double hoodAngleDegs = 13;
  private double turretTargetDegs = 180.0;
  private double turretDriveAssistTargetAngle = 0;
  private double hoodTargetDegs = 13.0;
  private double verticalTrim = 0.0;
  private double horizontalTrim = 0.0;

  private boolean emergencyStopSwivel = false;

  private boolean shooting = false;

  private final PIDController turretPID = new PIDController(1.5, 0.0, 0);
  private final PIDController hoodPID = new PIDController(0.8, 0.0, 0);

  public ShooterSim() {
    register();

    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("turretRoot", 1.5, 0.5);

    turretBase =
        root.append(new MechanismLigament2d("TurretBase", 0.6, 0, 12, new Color8Bit(Color.kBlue)));

    hood = turretBase.append(new MechanismLigament2d("Hood", 0.4, 0, 6, new Color8Bit(Color.kRed)));
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
    hood.setAngle(hoodAngleDegs);

    Logger.recordOutput("Shooter/Sim Turret Angle", turretAngleDegs);
    Logger.recordOutput("Shooter/Sim Hood Angle", hoodAngleDegs);
    Logger.recordOutput("Shooter/Sim Turret Target", turretTargetDegs);
    Logger.recordOutput("Shooter/Sim Hood Target", hoodTargetDegs);
    Logger.recordOutput("Shooter/Drive Assist Angle", turretDriveAssistTargetAngle);
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
    shooting = rps >= 1;
  }

  public boolean isShooting() {
    return shooting;
  }

  @Override
  public void setTurretTarget(double angle) {
    double turretAngle =
        MathUtil.clamp(angle, Constants.SHOOTER_TURRET_MIN_YAW, Constants.SHOOTER_TURRET_MAX_YAW);
    if (Math.abs(angle - turretAngle) > 2.0) {
      turretDriveAssistTargetAngle = MathUtil.inputModulus(angle - turretAngle, -180, 180);
    } else {
      turretDriveAssistTargetAngle = 0;
    }
    turretTargetDegs = turretAngle + horizontalTrim;
  }

  @Override
  public void setHoodTarget(double hoodTargetAngle) {
    hoodTargetDegs = hoodTargetAngle + verticalTrim;
  }

  @Override
  public void zeroHood(double newValue) {
    // not relevant in sim
  }

  @Override
  public double getDriveAssistanceAngle() {
    return turretDriveAssistTargetAngle;
  }

  @Override
  public void setSwivelStop(boolean stopped) {
    emergencyStopSwivel = stopped;
  }

  @Override
  public void adjustVerticalTrim(double trim) {
    if (trim == 1) {
      verticalTrim += Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else if (trim == -1) {
      verticalTrim -= Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else {
      verticalTrim = 0;
    }
  }

  @Override
  public void adjustHorizontalTrim(double trim) {
    if (trim == 1) {
      horizontalTrim += Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else if (trim == -1) {
      horizontalTrim -= Constants.SHOOTER_TRIM_ADJUSTMENT_INCREMENT;
    } else {
      horizontalTrim = 0;
    }
  }
}
