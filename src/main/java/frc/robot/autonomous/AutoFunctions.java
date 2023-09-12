package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.command.PurpleLed;
import frc.robot.subsystems.leds.command.YellowLed;
import frc.robot.utils.controllers.DieterController;

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

public class AutoFunctions extends SequentialCommandGroup {
    protected final Gripper gripper = Gripper.getInstance();
    protected final DieterController yawController = new DieterController(3, 0, 0, 0);
    protected final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    protected final Gyroscope gyroscope = Gyroscope.getInstance();


    protected CommandBase engage(boolean forwards, boolean returnArm) {
        return new SequentialCommandGroup(
                new DriveTillPitch(-10.5 * direction(forwards), 1.5 * direction(forwards))
                        .alongWith(returnArm ? new ReturnArm().withTimeout(0.65) : new InstantCommand()),

                new RunCommand(() -> swerveDrive.drive(
                        1.5 * direction(forwards),
                        0,
                        yawController.calculate(gyroscope.getYaw().getRadians(), 0),
                        true

                ), swerveDrive).alongWith(new GetArmIntoRobot()).withTimeout(
                        forwards ?
                                SwerveConstants.FORWARD_BALANCE_TIME :
                                SwerveConstants.BACKWARD_BALANCE_TIME),

                new InstantCommand(swerveDrive::lock)
        );
    }


    protected int direction(boolean forwards) {
        return forwards ? 1 : -1;
    }


    protected CommandBase autoUpperScoring(boolean isCone) {
        return new SequentialCommandGroup(
                isCone ? new YellowLed() :
                        new PurpleLed(),
                isCone ? new UpperScoring().withTimeout(1.5) :
                        new UpperScoring().withTimeout(1.2),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(0.65)
        );
    }

    protected CommandBase autoBegin(PathPlannerTrajectory trajectory) {
        return new SequentialCommandGroup(
                FollowPath.resetCommand(swerveDrive, gyroscope).apply(trajectory).alongWith(
                        new InstantCommand(gripper::close, gripper)),
                new Retract(DOWN).withTimeout(0.35),
                autoUpperScoring(true),
                new InstantCommand(gripper::open, gripper),
                new ReturnArm().withTimeout(0.65),
                new PurpleLed());
    }

    protected CommandBase bumperTakeCube() {
        return FollowPath.loadTrajectory("BumperConeCubeHigh 1", FollowPath.resetCommand(swerveDrive, gyroscope))
                .alongWith(
                        new PickUpCube().withTimeout(3.5));
    }
}

