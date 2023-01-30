package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmJoystickControl extends CommandBase {
    private final Arm arm;
    private final Joystick shoulderJoystick;
    private final Joystick elbowJoystick;

    public ArmJoystickControl(Joystick shoulderJoystick, Joystick elbowJoystick) {
        this.arm = Arm.getInstance();
        this.shoulderJoystick = shoulderJoystick;
        this.elbowJoystick = elbowJoystick;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setShoulderJointPower(-shoulderJoystick.getY());
        arm.setElbowJointPower(-elbowJoystick.getY());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
