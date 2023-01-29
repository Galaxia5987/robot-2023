package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmJoystickControl extends CommandBase {
    private final Arm arm;
    private final Joystick joystick;

    public ArmJoystickControl(Joystick joystick) {
        this.arm = Arm.getInstance();
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setShoulderJointPower(-joystick.getY());
        arm.setElbowJointPower(-joystick.getY());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
