package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.commands.Retract;

public class FeedTeleop extends ParallelCommandGroup {
    public FeedTeleop(boolean outtake) {
        Intake intake = Intake.getInstance();

        addCommands(
                new Retract(Retract.Mode.DOWN).andThen(new RunCommand(() -> intake.setAngle(IntakeConstants.ANGLE_DOWN))),
                intake.run(outtake ? -IntakeConstants.INTAKE_POWER : IntakeConstants.INTAKE_POWER)
        );
    }
}

