package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeReal;

public class RunIntakeCommand extends Command {

    private final IntakeReal m_Real;

    public RunIntakeCommand(IntakeReal intake) {
         m_Real= intake;
    }

    public void execute(){
        m_Real.intakeActivating = true;
        m_Real.activateIntake();

    }


}
