package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class Add5ToElbowAngle extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private double angle;

    public Add5ToElbowAngle() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        angle = arm.getElbowJointAngle().getDegrees() + 5;
    }

    @Override
    public void execute() {
        arm.setElbowJointAngle(angle);
    }
}
