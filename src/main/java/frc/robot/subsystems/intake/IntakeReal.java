package frc.robot.subsystems.intake;

import frc.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeReal extends IntakeIO {

    private final TalonFX intakeMotor= new TalonFX(Constants.INTAKE_MOTOR_ID, CANBus.roboRIO());
    public static boolean intakeActivating= false;
    public static boolean isRejecting= false;

    public IntakeReal() {
      register();
    }

    public void activateIntake(){
        intakeMotor.set(1.0);
    }

    public void reverseIntake(){
        intakeMotor.set(-1);
    }

    public void stopIntake(){
        intakeMotor.set(0);
    }

  @Override
  public void periodic() {

  }
}
