package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class FeederPosition extends SequentialCommandGroup {
    private static Translation2d offset = new Translation2d(0, 0);

    public static void setOffset(Translation2d offset) {
        FeederPosition.offset = offset;
    }

    public FeederPosition() {
        Arm arm = Arm.getInstance();

        addCommands(
                new InstantCommand(() -> {
                    if (arm.getEndPosition().getX() < 0) {
                        setOffset(new Translation2d(0, -0.02));
                    } else {
                        setOffset(new Translation2d(0, 0.02));
                    }
                }),
                new SetArmsPositionAngular(() -> ArmConstants.FEEDER_POSITION
                        .plus(offset), 0)
        );
    }
}
