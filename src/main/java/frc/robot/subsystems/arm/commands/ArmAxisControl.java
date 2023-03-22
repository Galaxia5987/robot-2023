package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmAxisControl extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final double Xvalue;
    private final double Yvalue;
    private final double multiplier;
    private Translation2d position = new Translation2d(0, 0);

    private double shoulderFFMultiplier = 0.0;
    private double elbowFFMultiplier = -1.5;

    public ArmAxisControl(double multiplier, double Xvalue, double Yvalue) {
        this.Xvalue = Xvalue;
        this.Yvalue = Yvalue;
        this.multiplier = multiplier;
        addRequirements(arm);
    }

    public ArmAxisControl(double multiplier, double Xvalue, double Yvalue,
                          double elbowFFMultiplier, double shoulderFFMultiplier) {
        this(multiplier, Xvalue, Yvalue);
        this.elbowFFMultiplier = elbowFFMultiplier;
        this.shoulderFFMultiplier = shoulderFFMultiplier;
    }

    @Override
    public void initialize() {
        position = arm.getEndPosition();
    }

    @Override
    public void execute() {
        boolean passedMaximum = position.getNorm() >= ArmConstants.SHOULDER_ARM_LENGTH + ArmConstants.ELBOW_ARM_LENGTH - 0.1;

        if (!passedMaximum) {
            position = position.plus(new Translation2d(Xvalue * multiplier, Yvalue * multiplier));
            arm.setEndPosition(position, shoulderFFMultiplier, elbowFFMultiplier);
        }
    }


}
