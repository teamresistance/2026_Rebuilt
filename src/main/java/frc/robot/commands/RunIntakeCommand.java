package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeYabadabaReal;

public class RunIntakeCommand extends Command {

    private final IntakeYabadabaReal m_Real;

    public RunIntakeCommand(IntakeYabadabaReal intake) {
         m_Real= intake;
    }

    public void execute(){
        m_Real.intakeActivating = true;
        m_Real.activateIntake();

    }


}
