package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.Retract;

public class ReturnIntake extends SequentialCommandGroup {

    public ReturnIntake() {
        Intake intake = Intake.getInstance();

        addCommands(
                new Retract(true),
                intake.run(0)
        );
    }
}
