package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
  }

  @Override
  public boolean atShootingSetpoints() {
    return turretPID.atSetpoint() && hoodPID.atSetpoint();
  }

  @Override
  public void runFlywheelAtRPS(double rps) {}

  @Override
  public void setTurretTarget(double turretTargetAngle) {
    turretTargetDegs = turretTargetAngle;
  }

  @Override
  public void setHoodTarget(double hoodTargetAngle) {
    hoodTargetDegs = hoodTargetAngle;
  }

  @Override
  public void zeroHood(double newValue) {}
}
