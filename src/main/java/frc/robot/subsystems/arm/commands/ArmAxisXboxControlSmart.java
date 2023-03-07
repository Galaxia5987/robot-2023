package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Utils;

public class ArmAxisXboxControlSmart extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final XboxController xboxController;

    private double shoulderHoldAngle;
    private double elbowHoldAngle;
    private boolean lastJoysticksZero;

    public ArmAxisXboxControlSmart(XboxController xboxController) {
        this.xboxController = xboxController;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double powerX = xboxController.getLeftX();
        double powerY = xboxController.getRightY();
        powerY = MathUtil.applyDeadband(powerY, 0.2);
        powerX = MathUtil.applyDeadband(powerX, 0.2);
        boolean joysticksZero = Utils.epsilonEquals(powerX, 0) && Utils.epsilonEquals(powerY, 0);

        if ((!lastJoysticksZero && joysticksZero) || arm.changedToDefaultCommand() || Robot.justEnabled()) {
            shoulderHoldAngle = arm.getShoulderJointAngle().getDegrees();
            elbowHoldAngle = arm.getElbowJointAngle().getDegrees();
        }
        if (joysticksZero) {
            arm.setShoulderJointAngle(shoulderHoldAngle, 1);
            arm.setElbowJointAngle(elbowHoldAngle, 1);
        } else {
            arm.setVelocity(new Translation2d(
                    powerX * 0.1, powerY * 0.1
            ));
        }

        lastJoysticksZero = joysticksZero;
    }
}
