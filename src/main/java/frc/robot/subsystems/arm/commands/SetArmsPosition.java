package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class SetArmsPosition extends CommandBase {
    private final Arm arm;
    private final Translation2d position;

    public SetArmsPosition(Arm arm, Translation2d position) {
        this.arm = arm;
        this.position = position;
        addRequirements();
    }

    @Override
    public void initialize() {
        arm.setEndPosition(position);
    }

    @Override
    public boolean isFinished() {
        if (arm.getEndPosition()==position){
            return true;
        }
        return false;
    }
}
