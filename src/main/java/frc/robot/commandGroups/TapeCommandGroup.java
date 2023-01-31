package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;

public class TapeCommandGroup extends SequentialCommandGroup {
    public TapeCommandGroup() {
        addCommands(
                //TODO: add following path to reflective tape target - thank you Eitan!
        );
    }
}
