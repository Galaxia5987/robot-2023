package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmAxisXboxControlDumb extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final XboxController xboxController;
    private final Double multiplier;

    public ArmAxisXboxControlDumb(XboxController xboxController, double multiplier) {
        this.xboxController = xboxController;
        this.multiplier = multiplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        var currentPosition = arm.getEndPosition();
        if (currentPosition.getY()> ArmConstants.END_POSITION_LOWER_Y_LIMIT&&currentPosition.getY()<ArmConstants.END_POSITION_UPPER_Y_LIMIT&&currentPosition.getX()>ArmConstants.END_POSITION_LOWER_X_LIMIT&&currentPosition.getX()<ArmConstants.END_POSITION_UPPER_X_LIMIT)
            arm.setEndPosition(new Translation2d(currentPosition.getX()+ xboxController.getLeftX()*multiplier, currentPosition.getY()+xboxController.getRightY()*multiplier));
    }
}
