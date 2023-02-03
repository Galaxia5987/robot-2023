package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunAllBits extends SequentialCommandGroup {
    public RunAllBits() {
        addCommands(
                new CheckIntakeFlow(),
                new CheckArmPositions(),
                new CheckSwerve()
        );
    }
}
