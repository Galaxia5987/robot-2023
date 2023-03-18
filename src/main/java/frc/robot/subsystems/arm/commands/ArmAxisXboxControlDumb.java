package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.Utils;

public class ArmAxisXboxControlDumb extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final XboxController xboxController;
    private final Double multiplier;
    private final double value;
    private final double lastPositionSign = 0;
    private double shoulderHoldAngle;
    private double elbowHoldAngle;
    private boolean lastJoysticksZero;
    private Translation2d position = new Translation2d(0, 0);
    private Translation2d initialPosition = new Translation2d(0, 0);
    private boolean lastPassedMaximum = false;

    public ArmAxisXboxControlDumb(XboxController xboxController, double multiplier, double value) {
        this.xboxController = xboxController;
        this.multiplier = multiplier;
        this.value = value;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        var currentPosition = arm.getEndPosition();
        double powerX = xboxController.getLeftX();
        double powerY = -xboxController.getLeftY();
        powerY = MathUtil.applyDeadband(powerY, 0.2);
        powerX = MathUtil.applyDeadband(powerX, 0.2);
        boolean joysticksZero = Utils.epsilonEquals(powerX, 0) && Utils.epsilonEquals(powerY, 0);
        boolean passedMaximum = position.getNorm() > ArmConstants.SHOULDER_ARM_LENGTH + ArmConstants.ELBOW_ARM_LENGTH - 0.1;

//        if (xboxController.getRightBumper()){
//            powerX = value;
//        }
        if ((lastJoysticksZero && !joysticksZero)) {
            initialPosition = currentPosition;
            position = initialPosition;
        }
        if ((!lastJoysticksZero && joysticksZero) || arm.changedToDefaultCommand() || Robot.justEnabled() ||
                (!lastPassedMaximum && passedMaximum)) {
            shoulderHoldAngle = arm.getShoulderJointAngle().getDegrees();
            elbowHoldAngle = arm.getElbowJointAngle().getDegrees();
        }
        if (joysticksZero || passedMaximum) {
            arm.setShoulderJointAngle(shoulderHoldAngle, 0);
            arm.setElbowJointAngle(elbowHoldAngle, 0);
        } else {
            position = position.plus(new Translation2d(powerX * multiplier, powerY * multiplier));

            arm.setEndPosition(position, 1, 0.5);
        }

        lastJoysticksZero = joysticksZero;
        lastPassedMaximum = passedMaximum;
    }
}
