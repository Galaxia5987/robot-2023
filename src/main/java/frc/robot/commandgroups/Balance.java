package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitchAccurate;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;

public class Balance extends SequentialCommandGroup {
    Gyroscope gyroscope = Gyroscope.getInstance();
    SwerveDrive swerveDrive = SwerveDrive.getInstance();

    double xVelocity;
    double startingPitch;
    double pitchTolerance;

    public Balance(double xVelocity, double startingPitch, double pitchTolerance) {
        this.pitchTolerance = pitchTolerance;
        this.xVelocity = xVelocity;
        this.startingPitch = startingPitch;
        addCommands(
                new DriveTillPitch(startingPitch, xVelocity),
                new DriveTillPitchAccurate(0, xVelocity).until(() -> Math.abs(gyroscope.getPitch().getDegrees()) < pitchTolerance)
        );
    }


}