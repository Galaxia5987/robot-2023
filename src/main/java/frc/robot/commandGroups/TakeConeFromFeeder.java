package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class TakeConeFromFeeder extends SequentialCommandGroup {
    private final Gripper gripper = Gripper.getInstance();
    private final Arm arm = Arm.getInstance();

    public TakeConeFromFeeder() {
        addCommands(
                FollowPath.generatePathToAprilTag(new SwerveDrive(), Limelight.getInstance(), new Gyroscope())
                        .alongWith(new InstantCommand(gripper::open)),
                new SetArmsPosition(ArmConstants.FEEDER_POSITION),
                new InstantCommand(gripper::close),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
