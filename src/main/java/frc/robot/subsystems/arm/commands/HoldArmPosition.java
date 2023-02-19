package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class HoldArmPosition extends CommandBase {
    private final Arm arm = Arm.getInstance();

    private double shoulderAngle;
    private double elbowAngle;

    public HoldArmPosition() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        shoulderAngle = arm.getShoulderJointAngle().getDegrees();
        elbowAngle = arm.getElbowJointAngle().getDegrees();
    }

    @Override
    public void execute() {
        arm.setShoulderJointAngle(shoulderAngle);
        arm.setElbowJointAngle(elbowAngle);
    }
}
