package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.intake.Intake;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow() {
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();
        addCommands(
                new InstantCommand(()->intake.setPower(ConstantsIntake.INTAKE_POWER), intake)
                        .alongWith(new InstantCommand(()->intake.setAngle(ConstantsIntake.ANGLE_DOWN), intake)),

                new WaitCommand(1),

                new InstantCommand(()->intake.setPower(ConstantsIntake.INTAKE_POWER), intake)
                         .alongWith(new InstantCommand(()->intake.setAngle(ConstantsIntake.ANGLE_UP), intake)),

                                 new WaitCommand(1),

                new InstantCommand(()->intake.setPower(ConstantsIntake.INTAKE_POWER), intake)
                          .alongWith(new InstantCommand(()->intake.setAngle(ConstantsIntake.ANGLE_DOWN), intake)),

                new WaitCommand(1),

                new InstantCommand(()->intake.setPower(ConstantsIntake.INTAKE_POWER), intake)
                           .alongWith(new InstantCommand(()->intake.setAngle(ConstantsIntake.ANGLE_UP), intake)),

                new WaitCommand(1),

                new InstantCommand(gripper::open, gripper),

                new WaitCommand(1),

                new InstantCommand(gripper::close, gripper),

                new WaitCommand(1),

                new InstantCommand(gripper::open, gripper)
        );
    }
}

