package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.PrototypeArm;
import frc.robot.subsystems.arm.commands.SetElbowAngle;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;
import frc.robot.subsystems.arm.commands.XboxControl;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);

    private final PrototypeArm arm = new PrototypeArm();

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
//        arm.setDefaultCommand(new RunCommand(() -> System.out.println(xboxController.getLeftX()), arm));
        arm.setDefaultCommand(new XboxControl(arm, xboxController));
    }

    private void configureButtonBindings() {
        a.onTrue(new SetElbowAngle(arm, 70));
        b.onTrue(new SetElbowAngle(arm, 0));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
