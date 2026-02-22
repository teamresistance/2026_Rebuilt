package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class intakeReal extends SubsystemBase {

    private final TalonFX intakeMotor= new TalonFX(Constants.INTAKE_MOTOR_ID, CANBus.roboRIO());
    public static boolean intakeActivating= false;
    public static boolean isRejecting= false;
    public intakeReal(){

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

}
