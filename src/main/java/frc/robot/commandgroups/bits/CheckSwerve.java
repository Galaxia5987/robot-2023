package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class CheckSwerve extends SequentialCommandGroup {
    public CheckSwerve(SwerveDrive swerveDrive) {
        addCommands(
                new InstantCommand(swerveDrive::vroom).withTimeout(8),
                new ZeroPositionSwerve(swerveDrive)
        );
    }
}



