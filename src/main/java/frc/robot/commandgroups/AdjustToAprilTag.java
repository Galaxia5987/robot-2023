package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class AdjustToAprilTag extends SequentialCommandGroup {
    public AdjustToAprilTag(boolean rightSide, boolean useHorizontalOffset) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Limelight limelight = Limelight.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        addCommands(
                new ProxyCommand(() -> FollowPath.generatePathToAprilTag(swerveDrive, limelight, gyroscope, rightSide, useHorizontalOffset))
        );
    }
}
