package frc.robot.commandGroups.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;

public class RunAllBits extends SequentialCommandGroup {
    public RunAllBits() {
        addCommands(
                new CheckIntakeFlow(),
                new CheckArmPositions(),
                new CheckSwerve()
        );
    }
}
