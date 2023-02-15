package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.ArmXboxControl;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import frc.robot.subsystems.drivetrain.commands.XboxDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.Feed;
import frc.robot.subsystems.intake.commands.InitializeEncoder;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.intake.commands.XboxWristControl;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    //    private final Leds led = Leds.getInstance();
    private final Arm arm = Arm.getInstance();
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
    private final Trigger leftPOV = new Trigger(() -> xboxController.getPOV() == 270);
    private final Trigger rightPOV = new Trigger(() -> xboxController.getPOV() == 90);

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

       // swerveSubsystem.setDefaultCommand(
//               new XboxDrive(swerveSubsystem, xboxController)
//        );
//       intake.setDefaultCommand(new XboxWristControl(xboxController)
                arm.setDefaultCommand(new ArmXboxControl(xboxController)
                );
    }

    private void configureButtonBindings() {
        a.onTrue(new InstantCommand(arm::resetArmEncoders));
        b.whileTrue(new SetShoulderAngle(60));
//        x.onTrue(new SetShoulderAngle())

        //       b.onTrue(new InstantCommand(gripper::toggle, gripper));

//        rightJoystickTrigger.onTrue(new InstantCommand(gyroscope::resetYaw));
//        leftJoystickTrigger.whileTrue(new AdjustToTarget(false, false));
//        lb.whileTrue(new BalanceOnStation());
//
//        leftPOV.whileTrue(new AdjustToTarget(false, true));
//        rightPOV.whileTrue(new AdjustToTarget(true, true));
//
//        a.onTrue(new FloorScoring());
//        a.onTrue(new InstantCommand(gripper::toggle, gripper));
//        b.onTrue(new MidScoring());
//        y.onTrue(new UpperScoring());
//        x.onTrue(new PickFromFeeder(ArmConstants.FEEDER_POSITION, ArmConstants.FEEDER_POSITION, true));
//
//        xboxLeftTrigger.whileTrue(new Feed(0.5));
//        xboxRightTrigger.onTrue(new InstantCommand(limelight::togglePipeline));
//        rb.onTrue(new InstantCommand(gyroscope::resetYaw));
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
