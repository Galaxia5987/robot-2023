package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.intake.Intake;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow(Intake intake, BeamBreaker beamBreaker, Gripper gripper) {
        addCommands(
                new Feed(0.5, ConstantsIntake.ANGLE_DOWN).withTimeout(3),
                new WaitCommand(1),
                new Feed(0, ConstantsIntake.ANGLE_UP),
                new WaitCommand(1),
                new Feed(0, ConstantsIntake.ANGLE_DOWN),
                new WaitCommand(1),
                new Feed(0, ConstantsIntake.ANGLE_UP),
                new WaitCommand(1),
                new InstantCommand(gripper::open),
                new WaitCommand(1),
                new InstantCommand(gripper::close),
                new WaitCommand(1),
                new InstantCommand(gripper::open)
        );
    }
}

