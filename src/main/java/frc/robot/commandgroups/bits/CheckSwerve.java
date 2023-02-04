package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class CheckSwerve extends SequentialCommandGroup {
    public CheckSwerve() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        addCommands(
                new InstantCommand(swerveDrive::vroom, swerveDrive).withTimeout(8),
                new ZeroPositionSwerve().withTimeout(2)
        );
    }
}



