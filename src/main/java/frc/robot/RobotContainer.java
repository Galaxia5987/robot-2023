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
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.ui.JoystickMap;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;
import frc.robot.subsystems.arm.commands.ArmXboxControl;

import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;

public class RobotContainer {
    public static Gyroscope gyroscope = new Gyroscope();
    public static SwerveDrive swerveSubsystem = new SwerveDrive();
    private static RobotContainer INSTANCE = null;
    private final Gripper gripper = Gripper.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final Limelight limelight = Limelight.getInstance();
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);

    private Command teleopTargetAdjustCommand;

    private static final Arm arm = Arm.getInstance();

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
        arm.setDefaultCommand(new ArmXboxControl(arm, xboxController));
        swerveSubsystem.setDefaultCommand(
                new HolonomicDrive(
                        swerveSubsystem,
                        new JoystickMap(leftJoystick, rightJoystick)
                )
        );
    }

    private void configureButtonBindings() {
        rightTrigger.whileTrue(new ProxyCommand(() -> FollowPath.generatePathToAprilTag(
                swerveSubsystem, limelight, gyroscope
        )));
//        leftTrigger.onTrue(new InstantCommand(() -> gyroscope.resetYaw(Rotation2d.fromDegrees(180))));
//        leftTrigger.onTrue(new InstantCommand(() -> gyroscope.resetYaw(Rotation2d.fromDegrees(90))));
        leftTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
        a.onTrue(new SetShoulderAngle(arm, 30));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        var trajectory = PathPlanner.loadPath("Onward", 5, 3);
        return new FollowPath(
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
