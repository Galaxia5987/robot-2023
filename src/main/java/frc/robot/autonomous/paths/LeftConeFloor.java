package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.FloorScoring;
import frc.robot.subsystems.vision.Limelight;

/**
 * This
 *
 */
public class LeftConeFloor extends SequentialCommandGroup {

    public LeftConeFloor() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setTapeMiddlePipeline, limelight),
                new FloorScoring()
        );
    }
}
