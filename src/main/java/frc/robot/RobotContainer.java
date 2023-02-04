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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.drivetrain.commands.XboxDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
//    private static final Leds led = Leds.getInstance();
//    private static final Arm arm = Arm.getInstance();
    public static Gyroscope gyroscope = new Gyroscope();
    public static SwerveDrive swerveSubsystem = new SwerveDrive();
    private static RobotContainer INSTANCE = null;
    private final XboxController xboxController = new XboxController(0);
    private final Limelight limelight = Limelight.getInstance();
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);
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
//        swerveSubsystem.setDefaultCommand(
//                new JoystickDrive(swerveSubsystem, leftJoystick, rightJoystick)
//        );
        swerveSubsystem.setDefaultCommand(
                new XboxDrive(swerveSubsystem, xboxController)
        );
    }

    private void configureButtonBindings() {
        a.whileTrue(new ProxyCommand(() -> teleopTargetAdjustCommand));
        rb.onTrue(new InstantCommand(gyroscope::resetYaw));
        b.whileTrue(new BalanceOnStation());
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
