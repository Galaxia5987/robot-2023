package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.FloorScoring;

/**
 * This
 */
public class LeftConeFloor extends SequentialCommandGroup {
    public LeftConeFloor(){
        addCommands(
                new FloorScoring()
        );
    }
}
