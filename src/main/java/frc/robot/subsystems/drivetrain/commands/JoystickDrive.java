package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Joystick joystick1;
    private final Joystick joystick2;

    public JoystickDrive(SwerveDrive swerveDrive, Joystick joystick1, Joystick joystick2) {
        this.swerveDrive = swerveDrive;
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                joystick1.getY(),
                joystick1.getX(),
                joystick2.getX(),
                true
        );
    }
}
