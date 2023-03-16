package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

public class SafetyAuto extends SequentialCommandGroup {

    public SafetyAuto() {
        Gripper gripper = Gripper.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();

        addCommands(
                new InstantCommand(gripper::close),

                new Retract(DOWN).withTimeout(0.35),

                new AutonUpperScoring(true),

                new InstantCommand(gripper::open),

                new ReturnArm().withTimeout(1),

                new GetArmIntoRobot().withTimeout(1.5),

                new InstantCommand(gyroscope::resetYaw),

                new DriveTillPitch(10, 2).withTimeout(2.8)
        );
    }
}
