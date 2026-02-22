package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeYabadabaReal;
import org.littletonrobotics.junction.Logger;

public class ReverseIntakeCommand extends Command {

    private final IntakeYabadabaReal m_real;

    public ReverseIntakeCommand(IntakeYabadabaReal intake) {
         m_real= intake;

    }

    public void execute(){
        m_real.reverseIntake();
        m_real.isRejecting= true;
        Logger.recordOutput("Intake status: ", m_real.isRejecting);
    }

    }
