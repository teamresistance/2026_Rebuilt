package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeReal;

public class RunIntakeCommand extends Command {

    private final intakeReal m_Real;

    public RunIntakeCommand(intakeReal intake) {
         m_Real= intake;
    }

    public void execute(){
        m_Real.intakeActivating = true;
        m_Real.activateIntake();

    }


}
