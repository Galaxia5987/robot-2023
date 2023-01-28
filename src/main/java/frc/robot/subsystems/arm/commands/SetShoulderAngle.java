package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class SetShoulderAngle extends CommandBase {
    private final Arm arm;
    private final double angle;

    public SetShoulderAngle(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderJointAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderJointPower(0);
        arm.setElbowJointPower(0);
    }
}