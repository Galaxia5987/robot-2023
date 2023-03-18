package frc.robot.autonomous.paths;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;

public class Engage extends SequentialCommandGroup {

    public Engage(boolean forwards, boolean returnArm) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();

        addCommands(
                new DriveTillPitch(-10.5 * direction(forwards), 1.5 * direction(forwards))
                        .alongWith(returnArm ? new ReturnArm().withTimeout(1) : new InstantCommand()),

                new RunCommand(() -> swerveDrive.drive(
                        new DriveSignal(
                                1.5 * direction(forwards),
                                0,
                                0,
                                new Translation2d(),
                                true
                        )
                ), swerveDrive).alongWith(new GetArmIntoRobot()).withTimeout(
                        forwards ?
                                SwerveConstants.FORWARD_BALANCE_TIME :
                                SwerveConstants.BACKWARD_BALANCE_TIME),

                new RunCommand(swerveDrive::lock)
        );
    }

    private int direction(boolean forwards) {
        return forwards ? 1 : -1;
    }
}
