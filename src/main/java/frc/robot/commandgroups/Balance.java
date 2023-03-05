package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.intake.commands.Retract;

public class Balance extends SequentialCommandGroup {

    public Balance(double xVelocity) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();

        addCommands(
                new DriveTillPitch(Math.signum(xVelocity) * (-10.5), xVelocity)
                        .alongWith(new Retract(false)),

                new RunCommand(() -> swerveDrive.drive(
                        new DriveSignal(
                                xVelocity,
                                0,
                                0,
                                new Translation2d(),
                                true
                        )
                ), swerveDrive).withTimeout(1.45)
        );
    }
}
