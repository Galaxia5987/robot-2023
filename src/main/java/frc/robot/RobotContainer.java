package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.ui.JoystickMap;
import frc.robot.utils.ui.XboxMap;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class RobotContainer {
    public static Gyroscope gyroscope = new Gyroscope();
    public static SwerveDrive swerveSubsystem = new SwerveDrive();
    private static RobotContainer INSTANCE = null;

private static final Leds led = Leds.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final Limelight limelight = Limelight.getInstance();
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
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
        swerveSubsystem.setDefaultCommand(
                new HolonomicDrive(
                        swerveSubsystem,
                        new XboxMap(xboxController)
                )
        );
    }

    }
    private void configureButtonBindings() {
//        rightTrigger.onTrue(new InstantCommand(() -> teleopTargetAdjustCommand = FollowPath.generatePathToAprilTag(
//                swerveSubsystem, limelight, gyroscope
//        )));
//        rightTrigger.whileTrue(new ProxyCommand(() -> teleopTargetAdjustCommand));
//        leftTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
        a.onTrue(new InstantCommand(() -> teleopTargetAdjustCommand = FollowPath.generatePathToAprilTag(
                swerveSubsystem, limelight, gyroscope
        )));
        a.whileTrue(new ProxyCommand(() -> teleopTargetAdjustCommand));
        rb.onTrue(new InstantCommand(gyroscope::resetYaw));
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
