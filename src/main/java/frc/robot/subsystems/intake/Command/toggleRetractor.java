package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class toggleRetractor extends CommandBase {
    private Intake intake = Intake.getINSTANCE();

    public toggleRetractor(Intake intake){
    addRequirements(intake);
    }

    /**
     * change the current state of the retractor to the contradictory state
     */
    @Override
    public void initialize() {
        intake.toggleSolenoid();
    }


    @Override
    public void end(boolean interrupted) {

    }


}
