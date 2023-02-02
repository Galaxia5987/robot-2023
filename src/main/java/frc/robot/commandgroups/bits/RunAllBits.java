package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ports;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;

public class RunAllBits extends SequentialCommandGroup {
    public RunAllBits(SwerveDrive swerveDrive, Gripper gripper, Arm arm) {
        addCommands(
                new CheckIntakeFlow( gripper),
                new CheckArmPositions(arm),
                new CheckSwerve(swerveDrive)
        );
    }
}
