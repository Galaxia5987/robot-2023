package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class JoystickControl extends CommandBase {
    private final Arm arm;
    private final Joystick joystick;

    public JoystickControl(Arm arm, Joystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        arm.setShoulderJointPower(arm.deadBend(joystick.getY()));
        arm.setElbowJointPower(arm.deadBend(joystick.getY()));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderJointPower(0);
        arm.setElbowJointPower(0);
    }
}
