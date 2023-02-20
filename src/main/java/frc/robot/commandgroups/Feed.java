package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.commands.Retract;

public class Feed extends ParallelCommandGroup {

    public Feed(boolean outtake) {
        Intake intake = Intake.getInstance();

        addCommands(
                new Retract(false),
                intake.run(outtake ? -IntakeConstants.INTAKE_POWER : IntakeConstants.INTAKE_POWER)
        );
    }
}
