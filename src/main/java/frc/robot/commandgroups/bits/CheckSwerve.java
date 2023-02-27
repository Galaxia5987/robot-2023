package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class CheckSwerve extends SequentialCommandGroup {
    public CheckSwerve() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        addCommands(
                new RunCommand(swerveDrive::vroom, swerveDrive).withTimeout(5),
                new ZeroPositionSwerve().withTimeout(2)
        );
    }
}



