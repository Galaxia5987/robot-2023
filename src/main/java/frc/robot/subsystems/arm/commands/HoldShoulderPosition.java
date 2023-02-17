package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class HoldShoulderPosition extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private double angle;

    public HoldShoulderPosition() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        angle = arm.getShoulderJointAngle().getDegrees();
    }

    @Override
    public void execute() {
        arm.setShoulderJointAngle(angle);
    }
}
