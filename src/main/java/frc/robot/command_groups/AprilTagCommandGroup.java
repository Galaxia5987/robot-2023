package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class AprilTagCommandGroup extends SequentialCommandGroup {
    public AprilTagCommandGroup() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Limelight limelight = Limelight.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        addCommands(
                FollowPath.generatePathToAprilTag(swerveDrive, limelight, gyroscope)
        );
    }
}
