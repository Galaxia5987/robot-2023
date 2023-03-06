package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.paths.*;
import frc.robot.commandgroups.*;
import frc.robot.commandgroups.bits.RunAllBits;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.*;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.AdjustToTargetDumb;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Utils;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    //    private final Leds led = Leds.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Leds leds = Leds.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final SwerveDrive swerveSubsystem = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Intake intake = Intake.getInstance();
   private final Gripper gripper = Gripper.getInstance();
    private final BeamBreaker beamBreaker = BeamBreaker.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton y = new JoystickButton(xboxController, XboxController.Button.kY.value);
    private final JoystickButton x = new JoystickButton(xboxController, XboxController.Button.kX.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final Trigger xboxRightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.2);
    private final Trigger xboxLeftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.2);
    private final JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TRIGGER);
    private final JoystickButton leftJoystickTopBottom = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TOP_BOTTOM_BUTTON);
    private final JoystickButton rightJoystickTrigger = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TRIGGER);
    private final JoystickButton rightJoystickTopBottom = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_BOTTOM_BUTTON);
    private final JoystickButton rightJoystickTopRight = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_RIGHT_BUTTON);
    private final JoystickButton rightJoystickTopLeft = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_LEFT_BUTTON);
    private final Trigger leftPOV = new Trigger(() -> xboxController.getPOV() == 270);
    private final Trigger rightPOV = new Trigger(() -> xboxController.getPOV() == 90);
    private final Trigger upPOV = new Trigger(() -> Utils.epsilonEquals(xboxController.getPOV(), 0));
    private final Trigger downPOV = new Trigger(() -> Utils.epsilonEquals(xboxController.getPOV(), 180));
    private final JoystickButton start = new JoystickButton(xboxController, XboxController.Button.kStart.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        // Configure the button bindings and default commands
        DriverStation.silenceJoystickConnectionWarning(true);

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
                new JoystickDrive(leftJoystick, rightJoystick)
        );
        arm.setDefaultCommand(new ArmXboxControl(xboxController));
//        arm.setDefaultCommand(new ArmXboxControl(xboxController));

    }

    private void configureButtonBindings() {

        b.whileTrue(new FeederPosition()
                .alongWith(new ReturnIntake()));
        y.whileTrue(new UpperScoring()
                .alongWith(new ReturnIntake()));
        x.whileTrue(new MidScoring()
                .alongWith(new ReturnIntake()));
        a.whileTrue(new ReturnArm());
        lb.onTrue(new InstantCommand(gripper::toggle));
        leftPOV.whileTrue(new ArmAxisControl(0.3, 0.02,0));
        rightPOV.whileTrue(new ArmAxisControl(0.3, -0.02,0));
        downPOV.whileTrue(new ArmAxisControl(0.3, 0,-0.01));
        upPOV.whileTrue(new ArmAxisControl(0.3, 0,0.01));

        xboxLeftTrigger.whileTrue(new PickUpCube())
                .onFalse(new ReturnIntake());
        xboxRightTrigger.whileTrue(new ReturnIntake()
                .andThen(new RunCommand(() -> intake.setAnglePower(0.05))))
                .onFalse(new InstantCommand(() -> intake.setAnglePower(0)));

        start.onTrue(new InstantCommand(leds::toggle));

        rightJoystickTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
        rightJoystickTopBottom.whileTrue(new AdjustToTargetDumb(
                AdjustToTargetDumb.Position.MIDDLE
        ));
        rightJoystickTopLeft.whileTrue(new AdjustToTargetDumb(
                AdjustToTargetDumb.Position.LEFT
        ));
        rightJoystickTopRight.whileTrue(new AdjustToTargetDumb(
                AdjustToTargetDumb.Position.RIGHT
        ));
    }


    /**
     *
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new Test();
    }
}
