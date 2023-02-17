package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class HoldElbowPosition extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private double angle;

    public HoldElbowPosition() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        angle = arm.getElbowJointAngle().getDegrees();
    }

    @Override
    public void execute() {
        arm.setElbowJointAngle(angle);
    }
}
