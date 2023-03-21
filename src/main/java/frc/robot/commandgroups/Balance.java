package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitchAccurate;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class Balance extends SequentialCommandGroup {
    Gyroscope gyroscope = Gyroscope.getInstance();
    SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public Balance(double xVelocityStart, double xVelocityEngage, double startingPitch) {
        addCommands(
                new DriveTillPitch(startingPitch, xVelocityStart),
                new DriveTillPitchAccurate(-4, xVelocityEngage)
        );
    }


}