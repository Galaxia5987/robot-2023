package frc.robot.commandGroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.Intake;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow() {
        Intake intake = Intake.getInstance();
        BeamBreaker beamBreaker = BeamBreaker.getInstance();
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new Feed(0.5, intake, beamBreaker).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5, intake, beamBreaker).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5, intake, beamBreaker).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5, intake, beamBreaker).withTimeout(3),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::close, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper)
        );
    }
}

