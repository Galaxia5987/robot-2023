package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.commands.AdjustToTarget;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.autonomous.FollowPath;
import frc.robot.autonomous.HolonomicFeedforward;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.ui.XboxMap;

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

    private void configureButtonBindings() {
        a.whileTrue(new AdjustToTarget(
                swerveSubsystem, gyroscope, limelight,
                SwerveConstants.TARGET_TRANSLATION_PID_CONSTANTS,
                SwerveConstants.TARGET_ROTATION_PID_CONSTANTS,
                new HolonomicFeedforward(SwerveConstants.TRANSLATION_FF_CONSTANTS))
        );
        rb.onTrue(new InstantCommand(gyroscope::resetYaw));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new FollowPath(
                swerveSubsystem, gyroscope,
                "Onward",
                SwerveConstants.AUTO_TRANSLATION_PID_CONSTANTS,
                SwerveConstants.AUTO_ROTATION_PID_CONSTANTS,
                new HolonomicFeedforward(SwerveConstants.TRANSLATION_FF_CONSTANTS, SwerveConstants.ROTATION_FF_CONSTANTS),
                4, 2);
    }
}
