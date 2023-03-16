package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.arm.commands.SetElbowAngle;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;
import frc.robot.subsystems.intake.Intake;

public class ReturnArm extends SequentialCommandGroup {

    public ReturnArm() {
        Arm arm = Arm.getInstance();
        var solution = arm.getKinematics().inverseKinematics(ArmConstants.OUT_ROBOT2);
        addCommands(
                new ConditionalCommand(
                        new SetArmsPositionAngular(() -> ArmConstants.OUT_ROBOT2),
                        new SequentialCommandGroup(
                                new SetShoulderAngle(Math.toDegrees(solution.shoulderAngle))
                                        .raceWith(new RunCommand(() -> Intake.getInstance().setAnglePower(0.05))
                                                .finallyDo((b) -> Intake.getInstance().setAnglePower(0)))
                                        .withTimeout(0.7),
                                new SetElbowAngle(Math.toDegrees(solution.elbowAngle))
                                        .raceWith(new RunCommand(() -> Intake.getInstance().setAnglePower(0.05))
                                                .finallyDo((b) -> Intake.getInstance().setAnglePower(0)))
                        ),
                        () -> ((arm.getEndPosition().getY() < 0) && (arm.getShoulderJointAngle().getDegrees() > 90))
                                || (arm.getShoulderJointAngle().getDegrees() < 90)
                )
        );
    }
}
