package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;

public class AutonomousPaths {
    private final Arm arm;
    private final Gripper gripper;
    private final SwerveDrive swerveDrive;
    private final Intake intake;
    private final FollowPath followPath;

    public AutonomousPaths(Arm arm, Gripper gripper, SwerveDrive swerveDrive, Intake intake, FollowPath followPath) {
        this.arm = arm;
        this.gripper = gripper;
        this.swerveDrive = swerveDrive;
        this.intake = intake;
        this.followPath = followPath;
    }

    protected CommandBase placeUpperCone11FollowPathAndPickUpCube(String path){
        return new SequentialCommandGroup(
                
        )
    }
}
