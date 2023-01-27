package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.ui.JoystickMap;

import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;

public class RobotContainer {
    public static Gyroscope gyroscope = new Gyroscope();
    public static SwerveDrive swerveSubsystem = new SwerveDrive();
    private static RobotContainer INSTANCE = null;
    private final XboxController xboxController = new XboxController(0);
    private final Limelight limelight = Limelight.getInstance();
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);

    private PathPlannerTrajectory trajectory = PathPlanner.loadPath("Onward", 2, 1);
    private Command teleopTargetAdjustCommand;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
        swerveSubsystem.setDefaultCommand(
                new HolonomicDrive(
                        swerveSubsystem,
                        new JoystickMap(leftJoystick, rightJoystick)
                )
        );
    }

    private void configureButtonBindings() {
        rightTrigger.whileTrue(new InstantCommand(() -> {
            var aprilTag = limelight.getAprilTagTarget();
            var botPose = limelight.getBotPose();
            var currVelocity = swerveSubsystem.getSpeeds();
            if (aprilTag.isPresent() && botPose.isPresent()) {
                gyroscope.resetYaw(botPose.get().getRotation());
                var pStart = new PathPoint(
                        botPose.get().getTranslation(),
                        new Rotation2d(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond),
                        botPose.get().getRotation(),
                        Math.hypot(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond));
                var pEnd = new PathPoint(
                        aprilTag.get().getTranslation().toTranslation2d(),
                        aprilTag.get().getRotation().toRotation2d(),
                        aprilTag.get().getRotation().toRotation2d(),
                        0);
                trajectory = PathPlanner.generatePath(new PathConstraints(5, 3), false,
                        pStart, pEnd);
            } else {
                trajectory = new PathPlannerTrajectory();
            }
            int direction = limelight.getTagId() < 5 ? -1 : 1;
            teleopTargetAdjustCommand = new PPSwerveControllerCommand(
                    trajectory,
                    swerveSubsystem::getPose,
                    new PIDController(SwerveConstants.AUTO_XY_Kp, SwerveConstants.AUTO_XY_Ki, SwerveConstants.AUTO_XY_Kd),
                    new PIDController(SwerveConstants.AUTO_XY_Kp, SwerveConstants.AUTO_XY_Ki, SwerveConstants.AUTO_XY_Kd),
                    new PIDController(SwerveConstants.AUTO_ROTATION_Kp, SwerveConstants.AUTO_ROTATION_Ki, SwerveConstants.AUTO_ROTATION_Kd),
                    (speeds) -> swerveSubsystem.drive(new DriveSignal(
                            speeds.vxMetersPerSecond * direction,
                            speeds.vyMetersPerSecond * direction,
                            speeds.omegaRadiansPerSecond,
                            new Translation2d(), false)),
                    true,
                    swerveSubsystem
            );
        }).andThen(new ProxyCommand(() -> teleopTargetAdjustCommand)));
//        leftTrigger.onTrue(new InstantCommand(() -> gyroscope.resetYaw(Rotation2d.fromDegrees(180))));
//        leftTrigger.onTrue(new InstantCommand(() -> gyroscope.resetYaw(Rotation2d.fromDegrees(90))));
        leftTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        var trajectory = PathPlanner.loadPath("Onward", 5, 3);
        PathPlannerServer.sendActivePath(trajectory.getStates());
        return new PPSwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                swerveSubsystem.getKinematics(),
                new PIDController(SwerveConstants.AUTO_XY_Kp, SwerveConstants.AUTO_XY_Ki, SwerveConstants.AUTO_XY_Kd),
                new PIDController(SwerveConstants.AUTO_XY_Kp, SwerveConstants.AUTO_XY_Ki, SwerveConstants.AUTO_XY_Kd),
                new PIDController(SwerveConstants.AUTO_ROTATION_Kp, SwerveConstants.AUTO_ROTATION_Ki, SwerveConstants.AUTO_ROTATION_Kd),
                swerveSubsystem::setStates,
                true,
                swerveSubsystem
        );
    }
}
