package frc.robot.commandGroups.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class RunAllBits extends SequentialCommandGroup {
    public RunAllBits() {
        addCommands(
                new CheckIntakeFlow(),
                new CheckArmPositions(),
                new CheckSwerve()
        );
    }
}
