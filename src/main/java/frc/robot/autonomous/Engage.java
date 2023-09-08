package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.controllers.DieterController;

public class Engage extends SequentialCommandGroup {
    private final DieterController yawController = new DieterController(3, 0, 0, 0);

    public Engage(boolean forwards, boolean returnArm) {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();

        addCommands(
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
//
//                new Balance(1.5 * direction(forwards), 0.75 * direction(forwards), -10.5 * direction(forwards))
//                        .alongWith(new GetArmIntoRobot()), TODO: Check engage

                new InstantCommand(swerveDrive::lock)
        );
    }

    private int direction(boolean forwards) {
        return forwards ? 1 : -1;
    }
}
