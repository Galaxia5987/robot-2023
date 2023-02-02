package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class TapeCommandGroup extends SequentialCommandGroup {
    public TapeCommandGroup() {
        addCommands(
                FollowPath.generatePathToAprilTag(new SwerveDrive(), Limelight.getInstance(), new Gyroscope())
        );
    }
}
